"""
Boucle 1 kHz — port de controlTick() (FurutaPendulum.ino) + métriques.

L'ordre des opérations reproduit celui de l'ISR : lecture encodeurs, contrôle
sécurité, contrôleur, commande moteur. La commande calculée à un tick n'agit
sur la physique qu'au tick suivant, exactement comme sur la machine (le timer
FOC applique ce que l'ISR a déposé).
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

from .agent import QLearning
from .classic import ControlClassic
from .fw_config import Cfg, Rig
from .physics import Encoders, Motor, Plant

FAULT_NAMES = {
    0: "none", 1: "alpha_dot", 2: "theta_dot",
    3: "theta_range", 4: "saturation", 5: "user_stop",
}

MODES = ("ql_train", "ql_greedy", "classic", "balance_only")


@dataclass
class EpisodeStats:
    index: int = 0
    reward: float = 0.0
    epsilon: float = 0.0
    duration: float = 0.0
    min_abs_alpha: float = math.pi   # meilleure remontée atteinte (0 = vertical)
    t_up: float = 0.0                # temps cumulé à moins de 10 deg du haut
    t_balance: float = 0.0           # plus longue tenue continue "équilibré"
    t_first_up: float = -1.0         # date de la 1re arrivée à <15 deg (-1 = jamais)
    t_locked: float = -1.0           # date où la tenue continue atteint 1 s
    max_tdot: float = 0.0
    max_adot: float = 0.0
    stuck_frac: float = 0.0          # part du temps où le bras est COLLÉ
    dead_action_frac: float = 0.0    # part des actions RL sous le décollement
    terminal: str = "timeout"
    faults: int = 0

    @property
    def swung_up(self) -> bool:
        return self.min_abs_alpha < math.radians(15)

    @property
    def success(self) -> bool:
        """Tenue verticale d'au moins une seconde : le seul vrai critère."""
        return self.t_balance >= 1.0


@dataclass
class RunSummary:
    episodes: list[EpisodeStats] = field(default_factory=list)
    sim_seconds: float = 0.0
    faults: int = 0

    def tail(self, n: int) -> list[EpisodeStats]:
        return self.episodes[-n:] if n < len(self.episodes) else self.episodes

    def success_rate(self, last: int = 50) -> float:
        e = self.tail(last)
        return sum(x.success for x in e) / len(e) if e else 0.0

    def swingup_rate(self, last: int = 50) -> float:
        e = self.tail(last)
        return sum(x.swung_up for x in e) / len(e) if e else 0.0

    def mean_reward(self, last: int = 50) -> float:
        e = self.tail(last)
        return sum(x.reward for x in e) / len(e) if e else 0.0

    def best_alpha(self) -> float:
        return min((x.min_abs_alpha for x in self.episodes), default=math.pi)


class Runner:
    def __init__(self, cfg: Cfg, rig: Rig, mode: str = "ql_train", seed: int = 1):
        if mode not in MODES:
            raise SystemExit(f"mode inconnu : {mode} (attendu : {', '.join(MODES)})")
        self.cfg = cfg
        self.rig = rig
        self.mode = mode
        self.rng = random.Random(seed)

        self.plant = Plant(cfg, rig)
        self.enc = Encoders(cfg, rig)
        self.motor = Motor(cfg)
        if int(getattr(cfg, "TC_SPLIT", 0)) > 0:
            from .tiles import SplitTileQLearning
            self.agent = SplitTileQLearning(cfg, self.rng)
        elif int(getattr(cfg, "TC_TILINGS", 0)) > 0:
            from .tiles import TileQLearning
            self.agent = TileQLearning(cfg, self.rng)
        else:
            self.agent = QLearning(cfg, self.rng)
        self.classic = ControlClassic(cfg)

        self.rl_counter = 0
        self.rl_u_cmd = 0.0
        self.rl_applied = 0.0
        self.t = 0.0
        self.fault = 0
        self.total_faults = 0

        self.ep = EpisodeStats(epsilon=self.agent.epsilon)
        self._balance_run = 0.0
        self._ticks = 0
        self._stuck_ticks = 0
        self._rl_steps = 0
        self._dead_steps = 0
        self._ep_t0 = 0.0
        self._fall_armed = False
        # Posé par un critère d'arrêt anticipé (cf. train.py --stop-when) :
        # inutile de laisser tourner des heures après convergence.
        self._stop = False

        if mode in ("ql_train", "ql_greedy"):
            self.agent.start_session(mode == "ql_greedy")
        else:
            self.classic.reset(mode == "balance_only")
            if mode == "balance_only":
                # Le pendule est placé à la main près du haut.
                self.plant.reset(alpha=math.radians(4))
                self.enc.calibrate_bottom()
                self.enc.alpha_prev = self.enc.alpha = math.radians(4)

    # ---- Sécurité : port de Safety::check ----
    def _safety(self) -> int:
        cfg, e = self.cfg, self.enc
        if abs(e.alpha_dot) > cfg.ALPHA_DOT_MAX:
            return 1
        if abs(e.theta_dot) > cfg.THETA_DOT_MAX:
            return 2
        if cfg.THETA_TURNS_MAX > 0.0 and abs(e.theta) > cfg.THETA_TURNS_MAX * 2.0 * math.pi:
            return 3
        if self.motor.is_saturated():
            self._sat_time = getattr(self, "_sat_time", 0.0) + cfg.CTRL_DT
            if self._sat_time > cfg.SAT_TIMEOUT_S:
                return 4
        else:
            self._sat_time = 0.0
        return 0

    def _recover_from_fault(self) -> None:
        """La machine passe en FAULT et attend l'utilisateur. En simu on
        redémarre le mode pour que la campagne aille au bout, mais la faute est
        comptée : un réglage qui faute sans arrêt est un mauvais réglage."""
        self.total_faults += 1
        self.ep.faults += 1
        self.motor.hard_stop()
        self.plant.reset()
        self.enc.calibrate_bottom()
        self.rl_applied = 0.0
        self.rl_u_cmd = 0.0
        self._sat_time = 0.0
        if self.mode in ("ql_train", "ql_greedy"):
            self.agent.end_episode()
            self.agent._begin_pause()
        else:
            self.classic.reset(self.mode == "balance_only")

    def _on_episode_start(self) -> None:
        """Appelé là où le firmware fait Encoders::rezeroArm(), au démarrage d'un
        épisode. C'est aussi le point d'accroche du reverse curriculum."""
        self.enc.rezero_arm(self.plant)
        frac = self.rig.curriculum_frac
        if frac <= 0.0 or self.rng.random() >= frac:
            self.agent.fall_limit = None      # épisode de swing-up normal
            self._fall_armed = False
            return
        # Épisode d'ÉQUILIBRE : on s'arrête dès que le pendule décroche, au lieu
        # de laisser l'épisode se transformer en swing-up et gaspiller 14 s.
        fall = self.rig.curriculum_fall_rad
        self.agent.fall_limit = fall if fall > 0.0 else None
        self._fall_armed = True
        # Place le pendule PRÈS DU HAUT, à l'arrêt. Il faut réaligner l'état du
        # dérivateur des encodeurs, sinon la différence finie voit un saut de
        # pi et sort une vitesse absurde au premier tick.
        # NE PAS toucher pend_zero : c'est la calibration du bas, la déplacer
        # ferait lire "en bas" un pendule physiquement en haut.
        # Position ET vitesse tirées au hasard près de l'équilibre : un départ
        # toujours identique n'apprend qu'un seul point de la zone.
        a0 = self.rng.uniform(-1.0, 1.0) * self.rig.curriculum_rad
        self.plant.al = a0
        self.plant.ald = self.rng.uniform(-1.0, 1.0) * self.rig.curriculum_adot
        self.plant.thd = 0.0
        self.enc.update(self.plant)          # alpha correct, vitesse aberrante
        self.enc.alpha_prev = self.enc.alpha  # ... qu'on efface juste après
        self.enc.theta_prev = self.enc.theta
        self.enc.alpha_dot_f = 0.0
        self.enc.theta_dot_f = 0.0
        self.enc.alpha_dot = 0.0
        self.enc.theta_dot = 0.0

    # ---- Un tick de 1 ms ----
    def tick(self) -> bool:
        """Retourne True si un épisode vient de se terminer."""
        cfg = self.cfg

        self.plant.step(self.motor.duty, cfg.CTRL_DT)
        self.enc.update(self.plant)
        self.t += cfg.CTRL_DT
        self._accumulate()

        self.fault = self._safety()
        if self.fault:
            fault_code = self.fault
            self._recover_from_fault()
            if self.mode in ("ql_train", "ql_greedy"):
                # Sur la machine une faute arrête la tentative. La simulation
                # redémarre automatiquement pour terminer une campagne, mais
                # elle doit tout de même clôturer et compter cet épisode :
                # auparavant plusieurs tentatives étaient fusionnées, ce qui
                # gonflait artificiellement les temps de tenue.
                self._close_episode(f"fault_{fault_code}")
                return True
            return False

        episode_end = False
        if self.mode in ("ql_train", "ql_greedy"):
            self.rl_counter += 1
            if self.rl_counter >= cfg.RL_DIVIDER:
                self.rl_counter = 0
                self.rl_u_cmd, episode_end = self.agent.step(
                    self.enc.theta, self.enc.alpha,
                    self.enc.theta_dot, self.enc.alpha_dot,
                    self._on_episode_start,
                )
                if not self.agent.paused:
                    self._rl_steps += 1
                    # « morte » = action NON NULLE trop faible pour décoller le
                    # bras. L'action neutre est un choix délibéré, pas un raté.
                    if 0.0 < abs(self.rl_u_cmd) < self.rig.breakaway_duty:
                        self._dead_steps += 1
            if self.agent.paused:
                self.motor.hard_stop()
                self.rl_applied = 0.0
            else:
                self.rl_applied += (self.rl_u_cmd - self.rl_applied) * (cfg.CTRL_DT / cfg.QL_U_TAU)
                self.motor.set_duty(self.rl_applied)
        else:
            self.motor.set_duty(self.classic.update(
                self.enc.theta, self.enc.alpha, self.enc.theta_dot, self.enc.alpha_dot))

        if episode_end:
            self._close_episode("timeout" if self.agent.steps_in_episode == 0
                                and self.ep.duration >= cfg.QL_EPISODE_S - 1e-6
                                else self._terminal_guess())
        return episode_end

    def _terminal_guess(self) -> str:
        cfg = self.cfg
        if self.ep.duration < cfg.QL_EPISODE_S - 0.2:
            after_up_fall = float(getattr(cfg, "QL_AFTER_UP_FALL_RAD", 0.0))
            arm_s = float(getattr(cfg, "QL_AFTER_UP_ARM_S", 0.0))
            if (after_up_fall > 0.0 and self.ep.t_first_up >= 0.0
                    and self.ep.t_balance >= arm_s
                    and abs(self.enc.alpha) > after_up_fall - 0.05):
                return "chute_apres_haut"
            # La chute d'un épisode d'équilibre se reconnaît en premier, sinon
            # elle était comptée comme "theta_turns" et le diagnostic mentait.
            if (self._fall_armed and self.rig.curriculum_fall_rad > 0.0
                    and abs(self.enc.alpha) > self.rig.curriculum_fall_rad - 0.05):
                return "chute"
            if abs(self.enc.theta_dot) > cfg.QL_TDOT_MAX - 0.5:
                return "theta_dot"
            return "theta_turns"
        return "timeout"

    def _accumulate(self) -> None:
        cfg, e = self.cfg, self.enc
        running = not (self.mode.startswith("ql") and self.agent.paused)
        if not running:
            return
        self._ticks += 1
        if self.plant.stuck:
            self._stuck_ticks += 1
        a = abs(e.alpha)
        self.ep.min_abs_alpha = min(self.ep.min_abs_alpha, a)
        if self.ep.t_first_up < 0.0 and a < math.radians(15):
            self.ep.t_first_up = self.t - self._ep_t0
        self.ep.max_tdot = max(self.ep.max_tdot, abs(e.theta_dot))
        self.ep.max_adot = max(self.ep.max_adot, abs(e.alpha_dot))
        if a < math.radians(10):
            self.ep.t_up += cfg.CTRL_DT
        # « Équilibré » = le pendule RESTE en haut. Le critère précédent exigeait
        # aussi |alpha'| < 1 rad/s à chaque tick de 1 ms et remettait le compteur
        # à zéro au moindre dépassement : or la seule quantification de
        # l'encodeur pendule produit des sauts de 1,57 rad/s sur la vitesse brute.
        # Un pendule visiblement stable pendant 13 s était donc compté 2,8 s.
        # On garde uniquement l'angle, avec la même limite que t_up.
        if a < math.radians(10):
            self._balance_run += cfg.CTRL_DT
            self.ep.t_balance = max(self.ep.t_balance, self._balance_run)
            if self.ep.t_locked < 0.0 and self._balance_run >= 1.0:
                self.ep.t_locked = self.t - self._ep_t0
        else:
            self._balance_run = 0.0
        self.ep.duration = self.t - self._ep_t0

    def _close_episode(self, terminal: str) -> None:
        self.ep.index = self.agent.episode
        self.ep.reward = self.agent.last_episode_reward
        self.ep.epsilon = self.agent.epsilon
        self.ep.terminal = terminal
        self.ep.stuck_frac = self._stuck_ticks / self._ticks if self._ticks else 0.0
        self.ep.dead_action_frac = self._dead_steps / self._rl_steps if self._rl_steps else 0.0
        self._finished = self.ep
        self.ep = EpisodeStats(epsilon=self.agent.epsilon)
        self._balance_run = 0.0
        self._ticks = self._stuck_ticks = self._rl_steps = self._dead_steps = 0
        self._ep_t0 = self.t

    # ---- Campagnes ----
    def run_episodes(self, n: int, on_episode=None) -> RunSummary:
        summary = RunSummary()
        # QL_FIRST_UP_BONUS_S peut prolonger une fois chaque épisode. Le garde
        # doit couvrir ce pire cas, sinon une bonne politique qui atteint le
        # sommet fait paradoxalement arrêter la campagne avant les n épisodes.
        max_episode_s = (self.cfg.QL_EPISODE_S
                         + max(0.0, float(getattr(
                             self.cfg, "QL_FIRST_UP_BONUS_S", 0.0))))
        guard = int(n * (max_episode_s + self.cfg.QL_SETTLE_MAX_S + 2.0)
                    / self.cfg.CTRL_DT) + 1000
        while len(summary.episodes) < n and guard > 0 and not self._stop:
            guard -= 1
            if self.tick():
                summary.episodes.append(self._finished)
                if on_episode:
                    on_episode(self._finished)
        summary.sim_seconds = self.t
        summary.faults = self.total_faults
        return summary

    def run_seconds(self, seconds: float) -> RunSummary:
        summary = RunSummary()
        for _ in range(int(seconds / self.cfg.CTRL_DT)):
            if self.tick():
                summary.episodes.append(self._finished)
        if not summary.episodes:          # modes classiques : un « épisode » unique
            self._close_episode("fin")
            summary.episodes.append(self._finished)
        summary.sim_seconds = self.t
        summary.faults = self.total_faults
        return summary
