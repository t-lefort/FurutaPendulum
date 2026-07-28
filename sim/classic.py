"""
Port de control_classic.cpp : swing-up énergétique + équilibre par retour d'état.

Sert de RÉFÉRENCE. Si le contrôleur classique n'arrive pas à relever le pendule
en simulation, c'est le modèle mécanique (ou le couple disponible) qui est en
cause, pas le Q-learning — inutile de régler l'agent avant d'avoir passé ce
test. C'est le même rôle que la Phase 5 du README sur la machine.
"""

from __future__ import annotations

import math

from .fw_config import Cfg

SWINGUP, BALANCE = 0, 1


class ControlClassic:
    def __init__(self, cfg: Cfg):
        self.cfg = cfg
        self.reset(False)

    def reset(self, balance_only: bool) -> None:
        self.balance_only = balance_only
        self.phase = BALANCE if balance_only else SWINGUP
        self.theta_int = 0.0
        self.kick_time = 0.0

    def update(self, theta, alpha, theta_dot, alpha_dot) -> float:
        cfg = self.cfg

        if self.phase == SWINGUP:
            if abs(alpha) < cfg.BAL_ENTER_RAD and abs(alpha_dot) < cfg.BAL_ENTER_ADOT:
                self.phase = BALANCE
        else:
            if abs(alpha) > cfg.BAL_EXIT_RAD:
                self.phase = BALANCE if self.balance_only else SWINGUP

        if self.phase == BALANCE:
            if self.balance_only and abs(alpha) > cfg.BAL_EXIT_RAD:
                self.theta_int = 0.0
                return 0.0

            i_term = 0.0
            if cfg.K_TH_I > 0.0 and abs(alpha) < cfg.BAL_ENTER_RAD:
                at_home = (abs(theta) < cfg.TH_I_DEAD_RAD
                           and abs(theta_dot) < cfg.TH_I_DEAD_DOT)
                if at_home:
                    self.theta_int -= self.theta_int * (cfg.CTRL_DT / cfg.TH_I_FADE_S)
                else:
                    self.theta_int += theta * cfg.CTRL_DT
                i_term = cfg.K_TH_I * self.theta_int
                if i_term > cfg.TH_I_MAX:
                    i_term = cfg.TH_I_MAX
                    self.theta_int = i_term / cfg.K_TH_I
                elif i_term < -cfg.TH_I_MAX:
                    i_term = -cfg.TH_I_MAX
                    self.theta_int = i_term / cfg.K_TH_I
            else:
                self.theta_int = 0.0

            return -(cfg.K_ALPHA * alpha + cfg.K_ADOT * alpha_dot
                     + cfg.K_TH * theta + cfg.K_THD * theta_dot + i_term)

        self.theta_int = 0.0

        # Amorçage : au repos exact, la loi d'énergie vaut 0 et rien ne démarre.
        if abs(alpha_dot) < cfg.SWING_KICK_ADOT and abs(alpha) > math.pi - cfg.SWING_KICK_RAD:
            self.kick_time += cfg.CTRL_DT
            half = cfg.SWING_KICK_HALF_S
            direction = 1.0 if math.fmod(self.kick_time, 2.0 * half) < half else -1.0
            return cfg.SWING_KICK_U * direction
        self.kick_time = 0.0

        e_top = cfg.eTop()
        energy = 0.5 * cfg.pendJ() * alpha_dot * alpha_dot + e_top * math.cos(alpha)
        return (cfg.KE_SWING * (energy - e_top) * alpha_dot * math.cos(alpha)
                - cfg.KTHD_SWING * theta_dot)
