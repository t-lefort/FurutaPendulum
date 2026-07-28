"""
Physique du pendule de Furuta + moteur + frottements + chaîne encodeur.

Équations du mouvement (Lagrange), alpha mesuré depuis la VERTICALE HAUTE pour
coller à la convention du firmware (alpha = 0 en haut, +/-pi en bas) :

    [J0 + Jp.sin²a]  th'' + m.La.l.cos(a) a''
        + 2.Jp.sin(a)cos(a).th'.a' - m.La.l.sin(a).a'²  = tau_axe
    m.La.l.cos(a) th'' + Jp a''
        - Jp.sin(a)cos(a).th'²      - m.g.l.sin(a)      = -frottement_pivot

avec J0 = J_bras + m.La² (inertie autour de l'axe vertical, rotor compris) et
Jp = inertie du pendule autour de SON pivot.

Intégration RK4 au pas de la boucle de contrôle (1 kHz), sans numpy : en scalaire
pur CPython est plus rapide que numpy pour un système de dimension 4.

Le frottement sec du train d'engrenages est modélisé en STICK-SLIP (seuil de
décollement statique, puis frottement dynamique), pas en simple visqueux. C'est
indispensable ici : tout le problème observé sur la machine est que les petites
actions du RL ne produisent aucun mouvement.
"""

from __future__ import annotations

import math
from math import cos, floor, sin

from .fw_config import Cfg, Rig

_TWO_PI = 2.0 * math.pi


def wrap_pi(a: float) -> float:
    """Identique à Encoders::wrapPi()."""
    while a > math.pi:
        a -= _TWO_PI
    while a < -math.pi:
        a += _TWO_PI
    return a


class Plant:
    """État physique VRAI (continu, non quantifié).

    theta : angle du bras, rad, cumulé.
    alpha : angle du pendule depuis la verticale haute, rad, CUMULÉ (non borné :
            le pendule peut faire des tours). Au repos il vaut pi.
    """

    def __init__(self, cfg: Cfg, rig: Rig):
        m = cfg.PEND_MASS
        self.Jp = rig.pend_inertia(cfg)
        self.J0 = rig.arm_inertia + m * rig.arm_len ** 2
        # Le signe porte la convention géométrique (cf. Rig.coupling_sign) : le
        # mirroir alpha -> -alpha se réduit exactement à mLal -> -mLal.
        self.mLal = rig.coupling_sign * m * rig.arm_len * cfg.PEND_LCOM
        self.mgl = m * cfg.G_GRAV * cfg.PEND_LCOM

        self.gear = cfg.MOTOR_GEAR_RATIO
        self.volt = cfg.MOTOR_VOLT_LIMIT
        self.kt = rig.motor_kt
        self.R = rig.motor_R
        # Couple disponible à l'axe, à l'arrêt, pour une duty de 1.0
        self.tau_per_duty = self.gear * self.kt * self.volt / self.R

        self.tau_static = rig.breakaway_duty * self.tau_per_duty
        self.tau_kin = rig.kinetic_ratio * self.tau_static
        self.b_th = rig.arm_viscous
        self.b_al = rig.pend_viscous
        self.c_al = rig.pend_coulomb

        self.v_stick = 0.02  # rad/s en dessous desquels on teste le collage

        # La matrice de masse doit être définie positive pour TOUT alpha, sinon
        # le système n'existe pas et l'intégration diverge en quelques ms au lieu
        # de produire un résultat visiblement faux. Le minimum du déterminant est
        # atteint en cos^2(alpha) = 1 : det_min = J0.Jp - (m.La.l)^2.
        det_min = self.J0 * self.Jp - self.mLal ** 2
        if det_min <= 0.0:
            raise SystemExit(
                "Paramètres mécaniquement impossibles : la matrice de masse n'est "
                f"pas définie positive (J0.Jp = {self.J0 * self.Jp:.3e} <= "
                f"(m.La.l)^2 = {self.mLal ** 2:.3e}).\n"
                f"  inertie pendule / pivot Jp = {self.Jp:.3e}, "
                f"m*lcom^2 = {m * cfg.PEND_LCOM ** 2:.3e}\n"
                "Un corps réel vérifie toujours Jp >= m*lcom^2 (théorème de "
                "Huygens). Si vous avez forcé pend_model=firmware, c'est "
                "attendu : la formule m*L^2/3 de Settings::pendJ() est "
                "incompatible avec PEND_LCOM. Utilisez pend_model=rod_bob."
            )

        self.reset()

    def reset(self, alpha: float | None = None) -> None:
        self.th = 0.0
        self.thd = 0.0
        self.al = math.pi if alpha is None else alpha
        self.ald = 0.0
        self.stuck = True

    # ---- Moteur : tension q -> couple à l'axe vertical ----
    def motor_tau(self, duty: float, thd: float) -> float:
        w_motor = self.gear * thd
        current = (duty * self.volt - self.kt * w_motor) / self.R
        return self.gear * self.kt * current

    # ---- Accélérations ----
    def _accel(self, thd, al, ald, duty, slip_dir):
        s = sin(al)
        c = cos(al)
        Jp = self.Jp

        tau = self.motor_tau(duty, thd) - self.b_th * thd
        if slip_dir:
            tau -= self.tau_kin * slip_dir

        a11 = self.J0 + Jp * s * s
        a12 = self.mLal * c
        rhs1 = tau - (2.0 * Jp * s * c * thd * ald - self.mLal * s * ald * ald)
        rhs2 = (Jp * s * c * thd * thd + self.mgl * s
                - self.b_al * ald - self.c_al * _sign(ald))

        det = a11 * Jp - a12 * a12
        return ((Jp * rhs1 - a12 * rhs2) / det,
                (a11 * rhs2 - a12 * rhs1) / det)

    def _accel_pend_only(self, al, ald):
        """Bras collé (th'' = 0) : le pendule oscille librement."""
        return (self.mgl * sin(al) - self.b_al * ald
                - self.c_al * _sign(ald)) / self.Jp

    def _hold_torque(self, al, ald):
        """Couple qu'il faut appliquer à l'axe pour maintenir th'' = 0.

        Le pendule qui oscille pousse sur le bras via le couplage inertiel ;
        le bras reste collé tant que le frottement statique peut encaisser cette
        réaction moins le couple moteur.
        """
        s, c = sin(al), cos(al)
        aldd = self._accel_pend_only(al, ald)
        return self.mLal * c * aldd - self.mLal * s * ald * ald

    # ---- Intégration ----
    def step(self, duty: float, dt: float) -> None:
        # L'état de collage est figé sur tout le pas : le réévaluer à l'intérieur
        # des sous-pas RK4 ferait broutter le solveur autour de th' = 0.
        if abs(self.thd) < self.v_stick:
            tau_m = self.motor_tau(duty, 0.0)
            if abs(self._hold_torque(self.al, self.ald) - tau_m) <= self.tau_static:
                self.thd = 0.0
                self.stuck = True
                self._step_pend_only(dt)
                return
        self.stuck = False

        slip_dir = _sign(self.thd)
        if slip_dir == 0.0:
            # Décollage depuis l'arrêt : le frottement s'oppose au couple moteur.
            slip_dir = _sign(self.motor_tau(duty, 0.0) - self._hold_torque(self.al, self.ald))

        th, thd, al, ald = self.th, self.thd, self.al, self.ald
        h = dt

        def f(_th, _thd, _al, _ald):
            a1, a2 = self._accel(_thd, _al, _ald, duty, slip_dir)
            return _thd, a1, _ald, a2

        k1 = f(th, thd, al, ald)
        k2 = f(th + .5 * h * k1[0], thd + .5 * h * k1[1],
               al + .5 * h * k1[2], ald + .5 * h * k1[3])
        k3 = f(th + .5 * h * k2[0], thd + .5 * h * k2[1],
               al + .5 * h * k2[2], ald + .5 * h * k2[3])
        k4 = f(th + h * k3[0], thd + h * k3[1],
               al + h * k3[2], ald + h * k3[3])

        h6 = h / 6.0
        new_thd = thd + h6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
        self.th += h6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.al += h6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
        self.ald += h6 * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3])

        # Le frottement sec ne doit jamais INVERSER le mouvement : s'il a fait
        # traverser zéro, le bras s'arrête (et recollera au pas suivant).
        if new_thd * thd < 0.0:
            new_thd = 0.0
        self.thd = new_thd

    def _step_pend_only(self, dt):
        al, ald = self.al, self.ald
        h = dt

        def f(_al, _ald):
            return _ald, self._accel_pend_only(_al, _ald)

        k1 = f(al, ald)
        k2 = f(al + .5 * h * k1[0], ald + .5 * h * k1[1])
        k3 = f(al + .5 * h * k2[0], ald + .5 * h * k2[1])
        k4 = f(al + h * k3[0], ald + h * k3[1])
        h6 = h / 6.0
        self.al += h6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
        self.ald += h6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])

    # ---- Diagnostics ----
    def energy(self) -> float:
        """Énergie du pendule seul, référence firmware : -mgl en bas, +mgl en haut."""
        return 0.5 * self.Jp * self.ald ** 2 + self.mgl * cos(self.al)

    def swing_speed(self) -> float:
        """|alpha'| nécessaire au point bas pour tout juste atteindre le haut."""
        return math.sqrt(4.0 * self.mgl / self.Jp)


def _sign(x: float) -> float:
    return 0.0 if x == 0.0 else (1.0 if x > 0.0 else -1.0)


# ============================================================
#  Chaîne de mesure — port fidèle de encoders.cpp
# ============================================================

class Encoders:
    """Quantification + différence finie + passe-bas, exactement comme le firmware.

    La quantification n'est pas cosmétique : à 1 kHz, un pas d'encodeur pendule
    (2pi/4000 rad) donne 1,57 rad/s de saut sur la vitesse brute. Après le filtre
    (VEL_FILT_ALPHA), il reste de l'ordre du bin de alphaDot — c'est du bruit qui
    tombe directement dans l'état du RL.
    """

    def __init__(self, cfg: Cfg, rig: Rig):
        self.cfg = cfg
        self.quantize = rig.quantize
        self.arm_cnt2rad = cfg.ARM_CNT2RAD
        self.pend_cnt2rad = cfg.PEND_CNT2RAD
        # rig.theta_sign : miroir entre la convention géométrique du simulateur et
        # celle de la machine (cf. Rig.theta_sign). Il MULTIPLIE ARM_SIGN, il ne
        # le remplace pas : ARM_SIGN reste le signe firmware validé au banc (+1).
        self.arm_sign = cfg.ARM_SIGN * rig.theta_sign
        self.pend_sign = cfg.PEND_SIGN
        self.dt = cfg.CTRL_DT
        self.filt = cfg.VEL_FILT_ALPHA
        self.arm_zero = 0
        self.pend_zero = 0
        self.calibrate_bottom()

    # -- compteurs matériels --
    def _raw_arm(self, plant: Plant) -> int:
        x = plant.th / self.arm_cnt2rad
        return int(floor(x)) if self.quantize else x

    def _raw_pend(self, plant: Plant) -> int:
        x = (plant.al - math.pi) / self.pend_cnt2rad
        return int(floor(x)) if self.quantize else x

    def calibrate_bottom(self) -> None:
        self.arm_zero = 0
        self.pend_zero = 0
        self.theta_prev = 0.0
        self.alpha_prev = math.pi
        self.theta_dot_f = 0.0
        self.alpha_dot_f = 0.0
        self.theta = 0.0
        self.alpha = math.pi
        self.theta_dot = 0.0
        self.alpha_dot = 0.0

    def rezero_arm(self, plant: Plant) -> None:
        """Encoders::rezeroArm() : offset logiciel, compteur matériel intact."""
        self.arm_zero = self._raw_arm(plant)
        self.theta_prev = 0.0

    def update(self, plant: Plant) -> None:
        theta = self.arm_sign * (self._raw_arm(plant) - self.arm_zero) * self.arm_cnt2rad
        alpha_cont = (self.pend_sign * (self._raw_pend(plant) - self.pend_zero)
                      * self.pend_cnt2rad + math.pi)
        alpha = wrap_pi(alpha_cont)

        theta_dot_raw = (theta - self.theta_prev) / self.dt
        alpha_dot_raw = wrap_pi(alpha - self.alpha_prev) / self.dt
        self.theta_prev = theta
        self.alpha_prev = alpha

        self.theta_dot_f += self.filt * (theta_dot_raw - self.theta_dot_f)
        self.alpha_dot_f += self.filt * (alpha_dot_raw - self.alpha_dot_f)

        self.theta = theta
        self.alpha = alpha
        self.theta_dot = self.theta_dot_f
        self.alpha_dot = self.alpha_dot_f


class Motor:
    """Port de Motor::setDuty() / hardStop() : écrêtage + limitation de pente."""

    def __init__(self, cfg: Cfg):
        self.cfg = cfg
        self.applied = 0.0
        self.on = False

    def set_duty(self, u: float) -> None:
        lim = self.cfg.DUTY_LIMIT
        u = max(-lim, min(lim, u))
        max_step = self.cfg.DUTY_SLEW_PER_S * self.cfg.CTRL_DT
        delta = u - self.applied
        self.applied += max(-max_step, min(max_step, delta))
        self.on = True

    def hard_stop(self) -> None:
        self.applied = 0.0
        self.on = False

    @property
    def duty(self) -> float:
        return self.applied if self.on else 0.0

    def is_saturated(self) -> bool:
        return abs(self.applied) >= self.cfg.DUTY_LIMIT - 1e-4
