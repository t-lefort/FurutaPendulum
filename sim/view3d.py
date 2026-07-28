"""
Vue 3D temps accéléré du pendule.

    python -m sim.view3d --speed 5
    python -m sim.view3d --mode classic --set KE_SWING=50
    python -m sim.view3d --mode ql_greedy --load-q runs/q.bin --speed 3

Sert à VOIR ce que les chiffres ne disent pas : est-ce que le bras broute, est-ce
que le pendule pompe ou est amorti, est-ce que l'agent tourne en rond. Pour
mesurer, utiliser `python -m sim.train` (bien plus rapide, sans rendu).

`--speed N` = N secondes simulées par seconde d'horloge. Au-delà de ~100, le
rendu devient le goulot : passer en headless.
"""

from __future__ import annotations

import argparse
import math
import sys

from .fw_config import load
from .runner import Runner
from .train import _kv

TRACE_LEN = 260


def build(args):
    cfg, rig = load(_kv(args.set), _kv(args.rig))
    for w in rig.consistency_report(cfg):
        print(f"/!\\ {w}\n", file=sys.stderr)
    runner = Runner(cfg, rig, mode=args.mode, seed=args.seed)
    if args.load_q:
        runner.agent.load_bin(args.load_q)
    return cfg, rig, runner


def main(argv=None) -> None:
    p = argparse.ArgumentParser(
        prog="python -m sim.view3d",
        description="Visualisation 3D du pendule de Furuta simulé.")
    p.add_argument("--mode", default="ql_train",
                   choices=["ql_train", "ql_greedy", "classic", "balance_only"])
    p.add_argument("--speed", type=float, default=3.0,
                   help="secondes simulées par seconde réelle")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("--set", action="append", metavar="CONST=VAL")
    p.add_argument("--rig", action="append", metavar="PARAM=VAL")
    p.add_argument("--load-q", metavar="FICHIER.bin")
    p.add_argument("--trace", action="store_true", default=True,
                   help="trace la trajectoire du bout du pendule")
    p.add_argument("--save", metavar="FICHIER.gif",
                   help="enregistre au lieu d'ouvrir une fenetre (GIF ou MP4)")
    p.add_argument("--seconds", type=float, default=8.0,
                   help="duree reelle enregistree avec --save")
    args = p.parse_args(argv)

    try:
        import matplotlib
        if args.save:
            matplotlib.use("Agg")   # avant pyplot, sinon le backend est deja fixe
        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation
    except ImportError:
        raise SystemExit(
            "matplotlib est requis pour la vue 3D :\n"
            "  pip install -r sim/requirements.txt\n"
            "(la mesure, elle, marche sans aucune dépendance : "
            "python -m sim.train)")

    cfg, rig, runner = build(args)
    La = rig.arm_len
    Lp = cfg.PEND_LEN
    reach = La + Lp

    fig = plt.figure(figsize=(11, 7))
    ax = fig.add_subplot(111, projection="3d")
    lim = reach * 1.06
    zlim = Lp * 1.20
    # Aspect PROPORTIONNEL aux plages de données : sans ça mplot3d étire l'axe z
    # et les angles affichés ne sont plus les angles réels — inexploitable pour
    # juger à l'oeil si le pendule passe la verticale.
    ax.set_box_aspect((1.0, 1.0, zlim / lim))
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(-zlim, zlim)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zticks([])
    ax.view_init(elev=22, azim=-58)

    # Repères fixes : plan du bras et cercle balayé par le pivot
    circle = [(La * math.cos(t / 60 * 2 * math.pi), La * math.sin(t / 60 * 2 * math.pi))
              for t in range(61)]
    ax.plot([c[0] for c in circle], [c[1] for c in circle], [0] * 61,
            color="0.75", lw=.8, zorder=1)
    ax.plot([0, 0], [0, 0], [-Lp * .35, Lp * .5], color="0.4", lw=6,
            solid_capstyle="round", zorder=2)

    arm_line, = ax.plot([], [], [], color="tab:blue", lw=5,
                        solid_capstyle="round", zorder=5)
    pend_line, = ax.plot([], [], [], color="tab:red", lw=3.5,
                         solid_capstyle="round", zorder=6)
    bob, = ax.plot([], [], [], "o", color="tab:red", ms=9, zorder=7)
    pivot, = ax.plot([], [], [], "o", color="k", ms=5, zorder=7)
    trace_line, = ax.plot([], [], [], color="tab:orange", lw=.9, alpha=.55, zorder=4)
    torque_line, = ax.plot([], [], [], color="tab:green", lw=3, zorder=5)

    hud = ax.text2D(0.015, 0.97, "", transform=ax.transAxes, family="monospace",
                    fontsize=9, va="top")
    title = ax.text2D(0.5, 0.97, "Pendule de Furuta — politique SARSA compacte",
                      transform=ax.transAxes, ha="center", va="top",
                      fontsize=12, weight="bold")
    banner = ax.text2D(0.5, 0.02, "", transform=ax.transAxes, ha="center",
                       fontsize=11, color="tab:red")

    trace: list[tuple[float, float, float]] = []
    ticks_per_frame = max(1, int(args.speed / args.fps / cfg.CTRL_DT))
    state = {"episodes": 0, "best": math.pi}

    def tip(th, al):
        """Bout du pendule. e_t = (-sin th, cos th, 0) ;
        direction du pendule = cos(alpha).ez + sin(alpha).et."""
        ct, st_ = math.cos(th), math.sin(th)
        s = Lp * math.sin(al)
        return (La * ct - s * st_, La * st_ + s * ct, Lp * math.cos(al))

    # La trace est échantillonnée DANS la boucle de ticks : un point par frame
    # donnerait une polyligne en zigzag dès qu'on accélère.
    trace_every = max(1, int(0.008 / cfg.CTRL_DT))

    def frame(_):
        for i in range(ticks_per_frame):
            if runner.tick():
                state["episodes"] += 1
                state["best"] = min(state["best"], runner._finished.min_abs_alpha)
                trace.clear()
            elif args.trace and i % trace_every == 0:
                trace.append(tip(runner.plant.th, runner.plant.al))

        th, al = runner.plant.th, runner.plant.al
        state["best"] = min(state["best"], abs(runner.enc.alpha))
        ct, st_ = math.cos(th), math.sin(th)
        px, py = La * ct, La * st_
        tx, ty, tz = tip(th, al)

        arm_line.set_data([0, px], [0, py])
        arm_line.set_3d_properties([0, 0])
        pend_line.set_data([px, tx], [py, ty])
        pend_line.set_3d_properties([0, tz])
        bob.set_data([tx], [ty])
        bob.set_3d_properties([tz])
        pivot.set_data([px], [py])
        pivot.set_3d_properties([0])

        # Flèche de couple : arc tangentiel proportionnel à la commande
        u = runner.motor.duty
        span = u * 1.2
        arc = [(La * math.cos(th + span * k / 12), La * math.sin(th + span * k / 12))
               for k in range(13)]
        torque_line.set_data([a[0] for a in arc], [a[1] for a in arc])
        torque_line.set_3d_properties([0] * 13)

        if args.trace:
            del trace[:-TRACE_LEN]
            trace_line.set_data([p[0] for p in trace], [p[1] for p in trace])
            trace_line.set_3d_properties([p[2] for p in trace])

        e = runner.enc
        lines = [
            f"t = {runner.t:7.1f} s   x{args.speed:g}   mode {args.mode}",
            f"theta  {e.theta / (2 * math.pi):+7.2f} tr   theta' {e.theta_dot:+7.2f} rad/s",
            f"alpha  {math.degrees(e.alpha):+7.1f} deg  alpha' {e.alpha_dot:+7.2f} rad/s",
            f"duty   {u:+7.3f}          {'BRAS COLLE' if runner.plant.stuck else ''}",
        ]
        if args.mode.startswith("ql"):
            a = runner.agent
            lines += [
                ("politique gloutonne" if a.greedy
                 else f"episode {a.episode:4d}   eps {a.epsilon:.3f}"),
                f"R_ep   {a.episode_reward:+8.1f}   action {a.last_action:+d}"
                f"  ({a.u_command:+.2f})",
                f"meilleure remontee : {math.degrees(state['best']):.1f} deg du haut",
            ]
        else:
            lines.append("phase " + ("BALANCE" if runner.classic.phase else "SWINGUP"))
        hud.set_text("\n".join(lines))

        if runner.mode.startswith("ql") and runner.agent.paused:
            banner.set_text("pause inter-episode — moteur coupe, on attend le repos")
        elif runner.fault:
            banner.set_text("FAUTE SECURITE")
        else:
            banner.set_text("")

        return (arm_line, pend_line, bob, pivot, trace_line, torque_line,
                hud, title, banner)

    # mplot3d laisse d'énormes marges par défaut : on remplit la fenêtre.
    fig.subplots_adjust(left=0.0, right=1.0, bottom=0.0, top=1.0)
    anim = FuncAnimation(fig, frame, interval=1000.0 / args.fps,
                         blit=False, cache_frame_data=False,
                         save_count=int(args.seconds * args.fps))
    if args.save:
        from pathlib import Path
        Path(args.save).parent.mkdir(parents=True, exist_ok=True)
        anim.save(args.save, fps=args.fps, dpi=90)
        print(f"{args.save} — {args.seconds:g} s a x{args.speed:g} "
              f"= {args.seconds * args.speed:g} s simulees")
    else:
        plt.show()


if __name__ == "__main__":
    main()
