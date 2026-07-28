"""
Entraînement accéléré et balayage de paramètres, sans affichage.

    python -m sim.train --check
    python -m sim.train --episodes 300 --plot runs/base.png --save-q runs/q.bin
    python -m sim.train --classic
    python -m sim.train --sweep QL_U_MIN=0.15,0.25,0.35 --episodes 200 --jobs 6
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time
from pathlib import Path

from .fw_config import Rig, load
from .runner import Runner

# Console Windows en cp1252 : ne jamais planter sur un caractère non représentable.
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(errors="replace")


class _Tee:
    """Duplique la sortie vers un fichier, pour pouvoir SUIVRE un balayage en
    cours depuis un autre terminal (`Get-Content -Wait`). Sans ça, une tâche de
    fond n'affiche rien avant sa toute fin."""

    def __init__(self, stream, path):
        self.stream = stream
        self.file = open(path, "w", encoding="utf-8", buffering=1)

    def write(self, s):
        self.stream.write(s)
        self.file.write(s)
        return len(s)

    def flush(self):
        self.stream.flush()
        self.file.flush()


# ============================================================
#  Audit statique : ce qui se voit sans rien simuler
# ============================================================

def audit(cfg, rig: Rig) -> None:
    from .physics import Plant

    plant = Plant(cfg, rig)
    m, lc = cfg.PEND_MASS, cfg.PEND_LCOM
    Jp = plant.Jp
    w0 = math.sqrt(m * cfg.G_GRAV * lc / Jp)
    w_swing = plant.swing_speed()

    print("=== Pendule ===")
    print(f"  modèle de masse         : {rig.pend_model}")
    print(f"  inertie / pivot         : {Jp:.4e} kg.m2  "
          f"(firmware pendJ() = {cfg.pendJ():.4e})")
    print(f"  fréquence propre en bas : {w0:.2f} rad/s = {w0 / (2 * math.pi):.2f} Hz "
          f"(demi-période {math.pi / w0 * 1000:.0f} ms)")
    print(f"  |alpha'| requis au point bas pour atteindre le haut : {w_swing:.1f} rad/s")

    print("=== Couple disponible à l'axe ===")
    tpd = plant.tau_per_duty
    print(f"  duty 1.0                : {tpd * 1000:.1f} mN.m")
    print(f"  QL_U_MAX = {cfg.QL_U_MAX:<5}       : {tpd * cfg.QL_U_MAX * 1000:.1f} mN.m")
    print(f"  décollement (mesuré)    : {plant.tau_static * 1000:.1f} mN.m "
          f"(duty {rig.breakaway_duty})")
    print(f"  couple utile net max    : "
          f"{(tpd * cfg.QL_U_MAX - plant.tau_static) * 1000:.1f} mN.m")
    print(f"  couple gravité pendule  : {m * cfg.G_GRAV * lc * 1000:.1f} mN.m (à l'horizontale)")
    print(f"  accél. max du bras      : "
          f"{(tpd * cfg.QL_U_MAX - plant.tau_static) / plant.J0:.0f} rad/s2")

    print("=== Discrétisation de l'état RL ===")
    da = 360.0 / cfg.QL_N_ALPHA
    dw = 2.0 * cfg.QL_ADOT_MAX / cfg.QL_N_ADOT
    print(f"  bin alpha               : {da:.2f} deg  "
          f"(zone du bonus +20 = +/-5 deg, soit {10 / da:.1f} bin)")
    print(f"  bin alpha'              : {dw:.2f} rad/s")
    q_step = cfg.PEND_CNT2RAD / cfg.CTRL_DT
    q_std = q_step / math.sqrt(6.0) * math.sqrt(
        cfg.VEL_FILT_ALPHA / (2.0 - cfg.VEL_FILT_ALPHA))
    print(f"  bruit de quantification sur alpha' : ~{q_std:.2f} rad/s eff. "
          f"({100 * q_std / dw:.0f} % d'un bin ; saut brut {q_step:.2f} rad/s)")
    if int(getattr(cfg, "TC_SPLIT", 0)) > 0:
        from .tiles import SplitTileQLearning
        split = SplitTileQLearning(cfg)
        print(f"  représentation          : tile coding factorisé "
              f"({split.global_coder.n_feat} globaux + "
              f"{split.local_coder.n_feat} locaux)")
        print(f"  poids SARSA             : {split.memory_bytes / 1024:.0f} kio "
              f"({100 * split.memory_bytes / (512 * 1024):.1f} % de RAM2)")
        # Portage prévu : une trace = index uint32_t + valeur float.
        trace_bytes = int(cfg.QL_TRACE_MAX) * 8
        spare = 512 * 1024 - split.memory_bytes - trace_bytes
        print(f"  traces (budget firmware): {trace_bytes / 1024:.1f} kio "
              f"({int(cfg.QL_TRACE_MAX)} entrées)")
        print(f"  marge RAM2 estimée      : {spare / 1024:.1f} kio "
              f"(hors éventuelles allocations dynamiques)")
    else:
        print(f"  taille de table         : "
              f"{cfg.QL_N_ALPHA * cfg.QL_N_ADOT * cfg.QL_N_ACT * 4 / 1024:.0f} kB")

    print("=== Apprentissage ===")
    horizon = 1.0 / (1.0 - cfg.QL_GAMMA)
    print(f"  pas RL                  : {1000 * cfg.RL_DT:.0f} ms ({1 / cfg.RL_DT:.0f} Hz)")
    print(f"  horizon gamma           : {horizon:.0f} pas = {horizon * cfg.RL_DT:.1f} s")
    n_eps = math.log(cfg.QL_EPS_MIN / cfg.QL_EPS0) / math.log(cfg.QL_EPS_DECAY)
    ep_wall = cfg.QL_EPISODE_S + cfg.QL_SETTLE_MAX_S
    print(f"  epsilon {cfg.QL_EPS0} -> {cfg.QL_EPS_MIN}       : {n_eps:.0f} épisodes "
          f"(~{n_eps * ep_wall / 3600:.1f} h sur la machine)")
    hold = cfg.QL_EXPLORE_HOLD * cfg.RL_DT
    print(f"  rafale d'exploration    : {1000 * hold:.0f} ms "
          f"({100 * hold / (math.pi / w0):.0f} % d'une demi-période)")
    rev = 2.0 * cfg.QL_U_MAX / cfg.DUTY_SLEW_PER_S
    print(f"  inversion pleine échelle: {1000 * rev:.0f} ms "
          f"(pas RL = {1000 * cfg.RL_DT:.0f} ms)")

    print("=== Jeu d'actions ===")
    from .agent import QLearning
    ag = QLearning(cfg)
    levels = [ag.action_u(i) for i in range(ag.n_act)]
    dead = [u for u in levels if 0 < abs(u) < rig.breakaway_duty]
    print("  " + "  ".join(f"{u:+.3f}" for u in levels))
    print(f"  actions sans effet (sous le décollement) : {len(dead)}/{ag.n_act - 1}")

    warnings = rig.consistency_report(cfg)
    print("=== Incohérences détectées ===")
    if not warnings:
        print("  (aucune)")
    for w in warnings:
        print(f"  /!\\ {w}")


# ============================================================
#  Une campagne
# ============================================================

def run_one(mode: str, episodes: int, seed: int,
            cfg_over: dict, rig_over: dict,
            load_q: str | None = None, verbose: bool = True,
            eval_every: int = 0, stop_when: float = 0.0,
            save_best_q: str | None = None):
    cfg, rig = load(cfg_over, rig_over)
    runner = Runner(cfg, rig, mode=mode, seed=seed)
    if load_q:
        runner.agent.load_bin(load_q)

    t0 = time.perf_counter()
    if mode in ("classic", "balance_only"):
        summary = runner.run_seconds(episodes * cfg.QL_EPISODE_S)
    else:
        best_probe = [-1.0]

        def progress(ep):
            if eval_every and ep.index > 0 and ep.index % eval_every == 0:
                su, hold = probe(cfg, rig, runner.agent)
                el = time.perf_counter() - t0
                print(f"  >> ep {ep.index:5d}  GLOUTON swing {100 * su:3.0f}%  "
                      f"tenue {hold:5.2f}s   ({el:.0f}s calcul, "
                      f"{ep.index * cfg.QL_EPISODE_S / 3600:.1f} h de banc)",
                      flush=True)
                # Les approximations de valeur peuvent regresser meme avec un
                # pas decroissant. Garder le meilleur probe evite qu'une bonne
                # politique soit ecrasee par la suite de l'entrainement.
                if save_best_q and su >= 0.99 and hold > best_probe[0]:
                    Path(save_best_q).parent.mkdir(parents=True, exist_ok=True)
                    runner.agent.save_bin(save_best_q)
                    best_probe[0] = hold
                    print(f"  >> meilleur checkpoint sauvegarde : "
                          f"{save_best_q} ({hold:.2f}s)", flush=True)
                if stop_when > 0.0 and su >= 0.99 and hold >= stop_when:
                    print(f"  >> critere atteint a l'episode {ep.index} — arret",
                          flush=True)
                    runner._stop = True
            if verbose and ep.index % 25 == 0:
                print(f"  ep {ep.index:4d}  R={ep.reward:9.1f}  "
                      f"eps={ep.epsilon:.3f}  "
                      f"min|a|={math.degrees(ep.min_abs_alpha):5.1f}deg  "
                      f"haut={ep.t_up:4.1f}s  equil={ep.t_balance:4.1f}s  "
                      f"colle={100 * ep.stuck_frac:3.0f}%  {ep.terminal}",
                      flush=True)
        summary = runner.run_episodes(episodes, on_episode=progress)
    wall = time.perf_counter() - t0
    return runner, summary, wall


def evaluate(runner, episodes: int = 30):
    """Évaluation GLOUTONNE, sans exploration ni apprentissage.

    C'est la seule métrique qui mesure la POLITIQUE. La récompense des derniers
    épisodes d'entraînement contient le bruit d'epsilon : elle baisse quand on
    explore plus, même si la politique s'améliore — comparer des réglages
    d'exploration sur cette base est trompeur.
    """
    runner.agent.greedy = True
    runner.agent.epsilon = 0.0
    runner.agent.explore_hold = 0
    # /!\ L'évaluation doit porter sur la VRAIE tâche : départ en bas, pas de
    # terminaison sur chute. Laisser le curriculum actif ferait démarrer une
    # partie des épisodes d'évaluation déjà près du haut — on mesurerait alors
    # une tâche plus facile que celle qu'on veut résoudre.
    runner.rig.curriculum_frac = 0.0
    runner.agent.fall_limit = None
    runner.agent._begin_pause()
    return runner.run_episodes(episodes)


def probe(cfg, rig, agent, episodes: int = 6):
    """Évaluation gloutonne SANS perturber l'entraînement en cours.

    Un second Runner partage le TABLEAU DE POIDS de l'agent entraîné (même objet,
    pas une copie) mais possède son propre état d'épisode. On peut donc mesurer
    la compétence en cours de route sans toucher aux traces, à epsilon ni au
    curriculum de l'entraînement.

    Sert à répondre en UN run à « combien d'épisodes faut-il ? », au lieu de
    relancer un entraînement complet par point de mesure — et c'est aussi le
    critère d'arrêt anticipé qui évitera de laisser la machine tourner des heures
    après convergence.
    """
    from copy import copy
    r2 = Runner(cfg, copy(rig), mode="ql_greedy", seed=12345)
    r2.rig.curriculum_frac = 0.0
    if hasattr(agent, "W"):
        r2.agent.W = agent.W          # partage, pas copie
    else:
        r2.agent.Q = agent.Q
    r2.agent.greedy = True
    r2.agent.epsilon = 0.0
    s = r2.run_episodes(episodes)
    holds = sorted((e.t_balance for e in s.episodes), reverse=True)
    return s.swingup_rate(episodes), (holds[len(holds) // 2] if holds else 0.0)


def evaluate_balance(runner, episodes: int = 20, rad: float = 0.15):
    """Évaluation gloutonne de l'ÉQUILIBRE SEUL : départ près du haut.

    Indispensable, et longtemps manquant : l'évaluation normale part du bas, donc
    si le swing-up échoue l'agent n'arrive jamais en haut et l'équilibre mesuré
    est 0 % MÊME S'IL EST PARFAITEMENT APPRIS. Les deux compétences doivent être
    mesurées séparément, sinon l'une masque l'autre.

    Pas de terminaison sur chute ici : on veut la durée de tenue, pas un épisode
    tronqué.
    """
    runner.agent.greedy = True
    runner.agent.epsilon = 0.0
    runner.agent.explore_hold = 0
    runner.rig.curriculum_frac = 1.0
    runner.rig.curriculum_rad = rad
    runner.rig.curriculum_fall_rad = 0.0     # 0 = pas de terminaison sur chute
    runner.agent.fall_limit = None
    runner.agent._begin_pause()
    return runner.run_episodes(episodes)


def report(summary, wall: float, label: str = "") -> None:
    eps = summary.episodes
    if not eps:
        print("aucun épisode terminé")
        return
    last = summary.tail(50)
    print(f"--- {label or 'résultat'} ---")
    print(f"  épisodes            : {len(eps)}")
    print(f"  temps simulé / réel : {summary.sim_seconds:.0f} s en {wall:.1f} s "
          f"(x{summary.sim_seconds / max(wall, 1e-9):.0f})")
    print(f"  récompense moy. (50 derniers) : {summary.mean_reward():.1f}")
    print(f"  meilleure remontée  : {math.degrees(summary.best_alpha()):.1f} deg du haut")
    ups = [e.t_first_up for e in eps if e.t_first_up >= 0.0]
    locks = [e.t_locked for e in eps if e.t_locked >= 0.0]
    print(f"  temps jusqu'au haut : "
          + (f"{sum(ups) / len(ups):.2f} s ({len(ups)}/{len(eps)} essais)"
             if ups else "jamais atteint"))
    print(f"  temps jusqu'a tenue : "
          + (f"{sum(locks) / len(locks):.2f} s ({len(locks)}/{len(eps)} essais)"
             if locks else "jamais verrouille"))
    print(f"  swing-up (50 der.)  : {100 * summary.swingup_rate():.0f} %")
    print(f"  équilibre >=1s      : {100 * summary.success_rate():.0f} %")
    print(f"  temps en haut moy.  : {sum(e.t_up for e in last) / len(last):.2f} s/épisode")
    print(f"  bras collé          : {100 * sum(e.stuck_frac for e in last) / len(last):.0f} %")
    print(f"  actions mortes      : "
          f"{100 * sum(e.dead_action_frac for e in last) / len(last):.0f} %")
    print(f"  |theta'| max        : {max(e.max_tdot for e in eps):.1f} rad/s")
    print(f"  |alpha'| max        : {max(e.max_adot for e in eps):.1f} rad/s")
    causes = {}
    for e in eps:
        causes[e.terminal] = causes.get(e.terminal, 0) + 1
    print(f"  fins d'épisode      : " + ", ".join(f"{k}={v}" for k, v in causes.items()))
    print(f"  fautes sécurité     : {summary.faults}")


def plot(summary, path: str) -> None:
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib absent : pas de graphique "
              "(pip install -r sim/requirements.txt)", file=sys.stderr)
        return

    eps = summary.episodes
    x = [e.index for e in eps]

    def rolling(v, k=20):
        out, acc = [], []
        for val in v:
            acc.append(val)
            if len(acc) > k:
                acc.pop(0)
            out.append(sum(acc) / len(acc))
        return out

    fig, ax = plt.subplots(2, 2, figsize=(12, 7))
    fig.suptitle("Q-learning — pendule de Furuta (simulation)")

    ax[0][0].plot(x, [e.reward for e in eps], lw=.6, alpha=.4, color="tab:blue")
    ax[0][0].plot(x, rolling([e.reward for e in eps]), color="tab:blue")
    ax[0][0].set_title("Récompense par épisode")
    ax[0][0].set_xlabel("épisode")
    ax[0][0].grid(alpha=.3)

    deg = [math.degrees(e.min_abs_alpha) for e in eps]
    ax[0][1].plot(x, deg, lw=.6, alpha=.4, color="tab:orange")
    ax[0][1].plot(x, rolling(deg), color="tab:orange")
    ax[0][1].axhline(15, ls="--", color="k", lw=.8, label="swing-up réussi")
    ax[0][1].set_title("Écart minimal à la verticale (deg, plus bas = mieux)")
    ax[0][1].set_xlabel("épisode")
    ax[0][1].invert_yaxis()
    ax[0][1].legend()
    ax[0][1].grid(alpha=.3)

    ax[1][0].plot(x, [e.t_up for e in eps], lw=.6, alpha=.4, color="tab:green",
                  label="temps à <10 deg")
    ax[1][0].plot(x, rolling([e.t_up for e in eps]), color="tab:green")
    ax[1][0].plot(x, rolling([e.t_balance for e in eps]), color="tab:red",
                  label="tenue continue max")
    ax[1][0].set_title("Temps passé en haut (s)")
    ax[1][0].set_xlabel("épisode")
    ax[1][0].legend()
    ax[1][0].grid(alpha=.3)

    ax[1][1].plot(x, [e.epsilon for e in eps], color="tab:purple", label="epsilon")
    ax[1][1].plot(x, [e.stuck_frac for e in eps], color="tab:brown",
                  lw=.8, label="bras collé")
    ax[1][1].plot(x, [e.dead_action_frac for e in eps], color="tab:gray",
                  lw=.8, label="actions mortes")
    ax[1][1].set_title("Exploration et frottement")
    ax[1][1].set_xlabel("épisode")
    ax[1][1].set_ylim(0, 1)
    ax[1][1].legend()
    ax[1][1].grid(alpha=.3)

    fig.tight_layout()
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    print(f"graphique -> {path}")


# ============================================================
#  Balayage
# ============================================================

def _sweep_worker(args):
    combo, mode, episodes, seed, base_cfg, base_rig = args
    cfg_over = dict(base_cfg)
    rig_over = dict(base_rig)
    rig_names = {f for f in Rig.__dataclass_fields__}
    for key, val in combo.items():
        (rig_over if key in rig_names else cfg_over)[key] = val
    runner, summary, wall = run_one(mode, episodes, seed, cfg_over, rig_over,
                                    verbose=False)
    ups = [e.t_first_up for e in summary.episodes if e.t_first_up >= 0.0]
    locks = [e.t_locked for e in summary.episodes if e.t_locked >= 0.0]
    is_ql = mode in ("ql_train", "ql_greedy")
    ev = evaluate(runner, 20) if is_ql else None
    # L'équilibre se mesure SÉPARÉMENT, sinon un swing-up raté le masque.
    evb = evaluate_balance(runner, 20) if is_ql else None
    hold = ([e.t_balance for e in evb.episodes] if evb else [])
    return combo, {
        "ev_swing": ev.swingup_rate(20) if ev else 0.0,
        "ev_succ": ev.success_rate(20) if ev else 0.0,
        "ev_deg": math.degrees(ev.best_alpha()) if ev else float("nan"),
        "bal_hold": sum(hold) / len(hold) if hold else 0.0,
        "bal_rate": evb.success_rate(20) if evb else 0.0,
        "reward": summary.mean_reward(),
        "swingup": summary.swingup_rate(),
        "success": summary.success_rate(),
        "best_deg": math.degrees(summary.best_alpha()),
        "t_up1": sum(ups) / len(ups) if ups else float("nan"),
        "t_lock": sum(locks) / len(locks) if locks else float("nan"),
        "t_up": sum(e.t_up for e in summary.tail(50)) / max(len(summary.tail(50)), 1),
        "stuck": sum(e.stuck_frac for e in summary.tail(50)) / max(len(summary.tail(50)), 1),
        "faults": summary.faults,
        "wall": wall,
    }


def sweep(specs: list[str], mode, episodes, seeds, jobs, base_cfg, base_rig) -> None:
    axes = {}
    for spec in specs:
        if "=" not in spec:
            raise SystemExit(f"--sweep attend NOM=v1,v2,... (reçu : {spec})")
        name, values = spec.split("=", 1)
        axes[name.strip()] = [v.strip() for v in values.split(",")]

    combos = [dict(zip(axes, vals)) for vals in itertools.product(*axes.values())]
    tasks = [(c, mode, episodes, s, base_cfg, base_rig)
             for c in combos for s in seeds]
    print(f"{len(combos)} configurations x {len(seeds)} graine(s) "
          f"x {episodes} épisodes\n")

    t0 = time.perf_counter()
    results = []
    total = len(tasks)

    def progress(combo, res):
        """Une ligne par tâche terminée, AU MOMENT où elle termine.

        Avec pool.map() le balayage restait muet jusqu'à ce que TOUTES les
        configurations finissent : sur un balayage de 15 min, zéro retour
        pendant 15 min. imap_unordered rend les résultats au fil de l'eau.
        """
        n = len(results)
        el = time.perf_counter() - t0
        eta = el / n * (total - n) if n else 0.0
        desc = " ".join(f"{k}={v}" for k, v in combo.items())
        print(f"[{n:>3}/{total}] {el:6.0f}s  {desc}  ->  "
              f"glouton swing {100 * res['ev_swing']:3.0f}% "
              f"equil {100 * res['ev_succ']:3.0f}%  "
              f"min|a| {res['ev_deg']:.1f}d   (ETA {eta:.0f}s)",
              flush=True)

    if jobs > 1:
        import multiprocessing as mp
        with mp.Pool(jobs) as pool:
            for combo, res in pool.imap_unordered(_sweep_worker, tasks):
                results.append((combo, res))
                progress(combo, res)
    else:
        for t in tasks:
            combo, res = _sweep_worker(t)
            results.append((combo, res))
            progress(combo, res)
    print()

    merged: dict[tuple, list] = {}
    for combo, res in results:
        merged.setdefault(tuple(sorted(combo.items())), []).append(res)

    # GLOUT.* = évaluation gloutonne post-entraînement : la vraie mesure de la
    # politique. Les colonnes "swing-up"/"équil." restent celles de
    # l'entraînement (bruit d'exploration inclus), gardées pour comparaison.
    header = list(axes) + ["EQ.tenue", "EQ.taux",
                           "GLOUT.swing", "GLOUT.équil", "GLOUT.min|a|",
                           "récomp.", "swing-up", "équil.", "min|a|",
                           "t_haut", "t_verrou", "haut(s)", "collé", "fautes"]
    rows = []
    for key, runs in merged.items():
        d = dict(key)
        avg = lambda k: sum(r[k] for r in runs) / len(runs)
        rows.append([d[a] for a in axes] + [
            f"{avg('bal_hold'):7.2f}s",
            f"{100 * avg('bal_rate'):6.0f} %",
            f"{100 * avg('ev_swing'):6.0f} %",
            f"{100 * avg('ev_succ'):6.0f} %",
            f"{avg('ev_deg'):6.1f}d",
            f"{avg('reward'):9.1f}",
            f"{100 * avg('swingup'):5.0f} %",
            f"{100 * avg('success'):5.0f} %",
            f"{avg('best_deg'):5.1f}d",
            f"{avg('t_up1'):6.2f}",
            f"{avg('t_lock'):7.2f}",
            f"{avg('t_up'):6.2f}",
            f"{100 * avg('stuck'):4.0f} %",
            f"{avg('faults'):5.1f}",
        ])
    # Tri sur l'évaluation GLOUTONNE (colonne 0 après les axes), pas sur la
    # récompense d'entraînement qui contient le bruit d'exploration.
    # Tri sur la tenue d'équilibre mesurée séparément, puis sur le swing-up.
    rows.sort(key=lambda r: (-float(r[len(axes)].rstrip("s")),
                             -float(r[len(axes) + 2].rstrip(" %"))))

    widths = [max(len(str(h)), max((len(str(r[i])) for r in rows), default=0))
              for i, h in enumerate(header)]
    line = lambda cells: "  ".join(str(c).rjust(w) for c, w in zip(cells, widths))
    print(line(header))
    print("-" * (sum(widths) + 2 * (len(widths) - 1)))
    for r in rows:
        print(line(r))
    print(f"\n{time.perf_counter() - t0:.0f} s de calcul "
          f"(classé par GLOUT.équil puis GLOUT.swing — évaluation gloutonne "
          f"sur 20 épisodes après entraînement)")


# ============================================================

def _kv(pairs: list[str]) -> dict:
    """NOM=VALEUR, répétable et/ou séparé par des virgules."""
    out = {}
    for group in pairs or []:
        for p in group.split(","):
            if not p.strip():
                continue
            if "=" not in p:
                raise SystemExit(f"attendu NOM=VALEUR, reçu : {p}")
            k, v = p.split("=", 1)
            out[k.strip()] = v.strip()
    return out


def main(argv=None) -> None:
    p = argparse.ArgumentParser(
        prog="python -m sim.train",
        description="Entraînement accéléré du pendule de Furuta "
                    "(constantes lues dans config.h).")
    p.add_argument("--mode", default="ql_train",
                   choices=["ql_train", "ql_greedy", "classic", "balance_only"])
    p.add_argument("--classic", action="store_true",
                   help="raccourci pour --mode classic (test de référence)")
    p.add_argument("--episodes", type=int, default=200)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--set", action="append", metavar="CONST=VAL",
                   help="surcharge une constante de config.h")
    p.add_argument("--rig", action="append", metavar="PARAM=VAL",
                   help="surcharge un paramètre machine (voir --check)")
    p.add_argument("--check", action="store_true",
                   help="audit statique des paramètres, sans simuler")
    p.add_argument("--plot", metavar="FICHIER.png")
    p.add_argument("--save-q", metavar="FICHIER.bin",
                   help="exporte la Q-table au format du firmware (carte SD)")
    p.add_argument("--save-best-q", metavar="FICHIER.bin",
                   help="sauvegarde le meilleur probe glouton intermediaire")
    p.add_argument("--load-q", metavar="FICHIER.bin")
    p.add_argument("--sweep", action="append", metavar="NOM=v1,v2,...")
    p.add_argument("--seeds", type=int, default=1,
                   help="nombre de graines par configuration en balayage")
    p.add_argument("--jobs", type=int, default=1)
    p.add_argument("--eval-every", type=int, default=0, metavar="N",
                   help="evaluation gloutonne tous les N episodes (0 = jamais)")
    p.add_argument("--stop-when", type=float, default=0.0, metavar="SECONDES",
                   help="arret des que la tenue gloutonne atteint SECONDES")
    p.add_argument("--log", metavar="FICHIER",
                   help="duplique la sortie dans un fichier, suivable en direct")
    args = p.parse_args(argv)

    if args.log:
        Path(args.log).parent.mkdir(parents=True, exist_ok=True)
        sys.stdout = _Tee(sys.stdout, args.log)
        print(f"# journal : {args.log}", flush=True)

    cfg_over, rig_over = _kv(args.set), _kv(args.rig)
    mode = "classic" if args.classic else args.mode

    if args.check:
        cfg, rig = load(cfg_over, rig_over)
        audit(cfg, rig)
        return

    if args.sweep:
        sweep(args.sweep, mode, args.episodes,
              list(range(args.seed, args.seed + args.seeds)),
              args.jobs, cfg_over, rig_over)
        return

    cfg, rig = load(cfg_over, rig_over)
    if mode.startswith("ql"):
        keys = (
            "QL_SARSA", "QL_LAMBDA", "QL_LR", "QL_LR_DECAY",
            "QL_LR_MIN", "QL_GAMMA",
            "QL_EPS_DECAY", "QL_EPS_TOP0", "QL_EPS_TOP_MIN",
            "QL_EPS_TOP_DECAY", "QL_EXPLORE_HOLD", "QL_EXPLORE_NEAR_RAD",
            "QL_EXPLORE_EPS_TOP",
            "QL_FIRST_UP_RAD", "QL_FIRST_UP_BONUS_S",
            "QL_AFTER_UP_FALL_RAD", "QL_AFTER_UP_ARM_S",
            "RL_DIVIDER", "QL_K_ENERGY", "QL_K_APPROACH",
            "QL_K_BAL", "QL_BAL_CONE_TDOT", "QL_K_TDOT_TOP",
            "QL_TDOT_TOP_RAD",
            "TC_SPLIT", "TC_GLOBAL_TILINGS", "TC_GLOBAL_N_ALPHA",
            "TC_GLOBAL_N_ADOT", "TC_GLOBAL_N_TDOT", "TC_GLOBAL_LR_SCALE",
            "TC_LOCAL_TILINGS", "TC_LOCAL_RAD", "TC_LOCAL_N_ALPHA",
            "TC_LOCAL_N_ADOT", "TC_LOCAL_N_TDOT", "TC_LOCAL_TDOT_MAX",
            "TC_LOCAL_LR_SCALE",
            "TC_SPLIT_OVERLAP",
        )
        print("# config RL : " + " ".join(
            f"{k}={getattr(cfg, k)}" for k in keys if hasattr(cfg, k)))
    for w in rig.consistency_report(cfg):
        print(f"/!\\ {w}\n")

    runner, summary, wall = run_one(mode, args.episodes, args.seed,
                                    cfg_over, rig_over, args.load_q,
                                    eval_every=args.eval_every,
                                    stop_when=args.stop_when,
                                    save_best_q=args.save_best_q)
    print()
    report(summary, wall, mode)
    if args.save_q:
        Path(args.save_q).parent.mkdir(parents=True, exist_ok=True)
        runner.agent.save_bin(args.save_q)
        if (int(getattr(cfg, "TC_TILINGS", 0)) > 0
                or int(getattr(cfg, "TC_SPLIT", 0)) > 0):
            # /!\ Format tile coding : PAS celui de storage.cpp. Le copier sur la
            # carte SD serait rejeté (magic différent) — ou pire, mal interprété.
            print(f"Poids tile coding -> {args.save_q}\n"
                  f"  /!\\ format PROPRE au tile coding, incompatible avec "
                  f"storage.cpp : ne pas copier sur la carte SD.")
        else:
            print(f"Q-table -> {args.save_q} "
                  f"(copier sous /q_current.bin sur la carte SD)")
    if args.plot:
        plot(summary, args.plot)


if __name__ == "__main__":
    main()
