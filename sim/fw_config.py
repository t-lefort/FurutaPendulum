"""
Constantes du firmware, lues DIRECTEMENT dans config.h.

La simulation ne redéfinit aucun réglage : elle parse `constexpr` de config.h
au démarrage. Un paramètre modifié dans le firmware est donc immédiatement pris
en compte ici, et une valeur validée en simulation est littéralement celle qui
part sur la Teensy. Pas de dérive possible entre les deux.

Les paramètres qui n'existent PAS dans config.h (géométrie du bras, constantes
moteur, frottements) vivent dans `Rig` : ce sont des propriétés de la machine
réelle, à mesurer. Voir sim/README.md.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass, fields
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
CONFIG_H = ROOT / "config.h"

_BLOCK_COMMENT = re.compile(r"/\*.*?\*/", re.S)
_LINE_COMMENT = re.compile(r"//.*")
_DECL = re.compile(
    r"^\s*constexpr\s+(?:unsigned\s+|signed\s+)?(\w+)\s+(\w+)\s*=\s*([^;]+);"
)
# suffixe flottant du C++ : 4000.0f -> 4000.0, sans toucher aux identifiants
_FLOAT_SUFFIX = re.compile(r"(?<=[\d.])[fF](?![\w.])")

_INT_TYPES = {
    "int", "long", "short", "size_t",
    "int8_t", "int16_t", "int32_t", "int64_t",
    "uint8_t", "uint16_t", "uint32_t", "uint64_t",
}


class Cfg(dict):
    """Constantes de config.h, accessibles en attribut : cfg.QL_U_MAX.

    `sync()` recopie les clés dans __dict__ pour que `cfg.X` soit une lecture
    d'attribut ORDINAIRE. Sans ça, chaque accès échoue d'abord la recherche
    normale puis retombe sur __getattr__ : mesuré à 4,6 millions d'appels pour
    un entraînement, purement gratuits. À rappeler après toute écriture.
    """

    def sync(self) -> "Cfg":
        self.__dict__.update(self)
        return self

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    # --- Dérivés de Settings::Data (settings.h) ---
    def eTop(self) -> float:
        return self.PEND_MASS * self.G_GRAV * self.PEND_LCOM

    def pendJ(self) -> float:
        """Inertie du pendule telle que la CALCULE LE FIRMWARE.

        Utilisée uniquement par le swing-up énergétique du mode classique. La
        simulation, elle, utilise l'inertie physique du modèle choisi dans Rig
        (voir Rig.pend_inertia) — l'écart entre les deux est justement une des
        choses que cette simu permet de mesurer.
        """
        if int(getattr(self, "PEND_J_ROD_BOB", 0)) > 0:
            return (self.PEND_MASS * self.PEND_LEN
                    * (4.0 * self.PEND_LCOM - self.PEND_LEN) / 3.0)
        return self.PEND_MASS * self.PEND_LEN * self.PEND_LEN / 3.0


def parse_config_h(path: Path = CONFIG_H) -> Cfg:
    text = _BLOCK_COMMENT.sub("", path.read_text(encoding="utf-8", errors="replace"))
    env = {"PI": math.pi, "TWO_PI": 2.0 * math.pi, "HALF_PI": math.pi / 2.0}
    cfg = Cfg()
    for raw in text.splitlines():
        m = _DECL.match(_LINE_COMMENT.sub("", raw))
        if not m:
            continue
        ctype, name, expr = m.groups()
        try:
            value = eval(_FLOAT_SUFFIX.sub("", expr).strip(), {"__builtins__": {}}, env)
        except Exception:
            continue  # expression non arithmétique (enum, cast…) : sans intérêt ici
        value = int(value) if ctype in _INT_TYPES else float(value)
        env[name] = value
        cfg[name] = value
    return cfg


# ============================================================
#  Paramètres de la MACHINE — absents de config.h, à mesurer
# ============================================================

@dataclass
class Rig:
    """Ce que le firmware ne sait pas et qu'il faut mesurer sur le montage.

    Les défauts sont des ordres de grandeur plausibles pour un GBM2804 et un
    bras de 10 cm, PAS des mesures. Tant qu'ils ne sont pas recalés, la simu
    dit si un jeu de paramètres RL est cohérent, pas s'il marchera au mm près.
    Priorité de recalage : breakaway_duty > motor_kt > arm_len > arm_inertia.
    """

    # --- Géométrie ---
    arm_len: float = 0.10        # m, axe vertical -> pivot du pendule
    arm_inertia: float = 3.0e-4  # kg.m^2 autour de l'axe vertical, ROTOR INCLUS
                                 # (inertie rotor x MOTOR_GEAR_RATIO^2)

    # --- Moteur BLDC piloté en tension q (SimpleFOC, mode couple) ---
    motor_R: float = 10.0        # ohm, résistance de phase
    # CALÉ sur le banc : avec kt = 0.100 et les réglages réels (KE_SWING = -50,
    # DutySlew 20, MOTOR_VOLT_LIMIT 6), le mode classique atteint le haut en
    # 1,15 s et verrouille l'équilibre en 2,22 s, puis tient — ce qui correspond
    # au comportement observé (< 3 s, sans "toupie").
    # A kt = 0.064 le bras manquait d'autorité : la loi d'énergie avait besoin
    # d'une dizaine de cycles et le bras partait en toupie avant de converger.
    motor_kt: float = 0.100      # N.m/A (= V.s/rad pour ke, unités SI)

    # --- Frottements du train d'engrenages (axe vertical) ---
    # Exprimés par la DUTY DE DÉCOLLEMENT, seule grandeur réellement mesurable :
    # menu Debug > Jog manuel, monter de 5 % en 5 % jusqu'à ce que le bras parte.
    breakaway_duty: float = 0.15
    kinetic_ratio: float = 0.75  # frottement dynamique / statique (< 1)
    arm_viscous: float = 3.0e-4  # N.m.s/rad

    # --- Frottements du pivot du pendule (roulement, faibles) ---
    pend_viscous: float = 2.0e-5
    pend_coulomb: float = 5.0e-6

    # --- Répartition de masse du pendule ---
    # rod_bob : tige uniforme + masse au bout, dosées pour retomber sur
    #           (PEND_MASS, PEND_LCOM, PEND_LEN). Le plus réaliste, et le seul
    #           des trois garanti physiquement cohérent.
    # point   : masse ponctuelle à PEND_LCOM au bout d'une tige sans masse.
    # firmware: la formule de Settings::pendJ() (tige uniforme, m.L^2/3).
    pend_model: str = "rod_bob"

    # --- Convention géométrique du couplage bras/pendule ---
    # De quel côté un alpha POSITIF penche, par rapport au sens qu'une duty
    # positive donne au bras. Mathématiquement c'est la substitution alpha ->
    # -alpha dans les équations, qui revient exactement à changer le signe du
    # terme de couplage m.La.lcom (les termes en sin/cos et la gravité suivent).
    #
    # /!\ CALIBRÉ SUR LA MACHINE, PAS DEVINÉ. Le montage réel fonctionne
    # (swing-up ET équilibre) avec PEND_SIGN = +1 et KE_SWING = -50 ; le signe
    # retenu ici est celui qui reproduit ça. Se tromper dessus rend les deux lois
    # du mode classique inversées EN SIMULATION et fait conclure à tort que le
    # firmware a un bug de signe.
    coupling_sign: float = -1.0

    # Miroir entre le sens de theta du simulateur et celui de la machine. Comme
    # coupling_sign, c'est du CALAGE, pas un réglage : le couple (theta_sign,
    # coupling_sign) = (-1, -1) est le seul des quatre qui reproduise le
    # comportement observé au banc (équilibre tenu indéfiniment avec KAlpha 9 /
    # KAdot 0.6 / KTh 0.2 / KThd 0.42). Multiplie ARM_SIGN dans Encoders, sans
    # le remplacer : ARM_SIGN reste le signe firmware, validé au banc à +1.
    #   (+1, +1) et (-1, +1) -> le pendule n'est pas rattrapé du tout (0,05 s)
    #   (+1, -1)             -> rattrapé puis perdu (0,48 s)
    #   (-1, -1)             -> tenu 120 s / 120 s  <- la machine
    theta_sign: float = -1.0

    # --- Aide à l'apprentissage, SIMULATION UNIQUEMENT ---
    # « Reverse curriculum » : fraction des épisodes qui démarrent le pendule
    # PRÈS DU HAUT au lieu de pendre en bas. Un système sous-actionné doit
    # normalement repartir de sa position d'équilibre à chaque épisode, ce qui
    # limite énormément l'exploration : l'agent ne VISITE quasiment jamais la
    # zone d'équilibre, donc ne peut pas apprendre à rattraper. La littérature
    # (EBERL, IEEE 2025) identifie exactement ce blocage.
    # /!\ Non transposable tel quel sur la machine (il faudrait placer le
    # pendule à la main). Sert à valider que la POLITIQUE est apprenable ; le
    # portage se ferait par un pré-entraînement en simu puis transfert de table.
    # L'épisode d'équilibre se TERMINE dès que |alpha| dépasse
    # curriculum_fall_rad : sinon il se transforme en épisode de swing-up
    # ordinaire et gaspille le reste des 15 s. C'est ce qui rend la zone
    # d'équilibre dense en échantillons (mesuré sans : 4 s de pompage pour 0,74 s
    # en haut, soit ~15 % des échantillons dans la zone qui compte).
    curriculum_frac: float = 0.0      # 0 = désactivé
    curriculum_rad: float = 0.25      # rad : |alpha| max au démarrage
    curriculum_adot: float = 1.0      # rad/s : |alpha'| max au démarrage
    curriculum_fall_rad: float = 0.60 # rad : au-delà, épisode terminé (0 = off)

    # --- Bruit de mesure ---
    quantize: bool = True        # quantification des encodeurs (1000 PPR x4)

    def pend_inertia(self, cfg: Cfg) -> float:
        """Inertie du pendule autour de SON PIVOT, kg.m^2."""
        m, lc, L = cfg.PEND_MASS, cfg.PEND_LCOM, cfg.PEND_LEN
        if self.pend_model == "point":
            return m * lc * lc
        if self.pend_model == "firmware":
            return m * L * L / 3.0
        if self.pend_model == "rod_bob":
            if not (L / 2.0 <= lc <= L):
                return m * lc * lc  # hors domaine : repli sur la masse ponctuelle
            m_rod = 2.0 * m * (1.0 - lc / L)
            m_bob = m - m_rod
            return m_rod * L * L / 3.0 + m_bob * L * L
        raise ValueError(f"pend_model inconnu : {self.pend_model}")

    def consistency_report(self, cfg: Cfg) -> list[str]:
        """Incohérences détectables entre config.h et la physique."""
        out = []
        m, lc, L = cfg.PEND_MASS, cfg.PEND_LCOM, cfg.PEND_LEN
        j_fw = cfg.pendJ()
        if j_fw < m * lc * lc:
            out.append(
                f"Settings::pendJ() = {j_fw:.3e} kg.m2 est INFÉRIEUR à m*lcom^2 = "
                f"{m * lc * lc:.3e} : impossible physiquement (inertie propre "
                f"négative). La formule m*L^2/3 suppose un centre de masse à "
                f"L/2 = {L / 2:.4f} m, or PEND_LCOM = {lc:.4f} m. Le swing-up "
                f"énergétique du mode classique travaille donc avec une inertie "
                f"fausse de {100 * (j_fw / self.pend_inertia(cfg) - 1):+.0f} %."
            )
        j = self.pend_inertia(cfg)
        w_swing = math.sqrt(4.0 * m * cfg.G_GRAV * lc / j)
        if w_swing > cfg.QL_ADOT_MAX:
            out.append(
                f"QL_ADOT_MAX = {cfg.QL_ADOT_MAX:.0f} rad/s est SOUS la vitesse "
                f"de passage au point bas exigée par un swing-up "
                f"({w_swing:.1f} rad/s) : les bins saturent au moment décisif."
            )
        if w_swing > cfg.ALPHA_DOT_MAX:
            out.append(
                f"ALPHA_DOT_MAX = {cfg.ALPHA_DOT_MAX:.0f} rad/s coupe en faute "
                f"AVANT la vitesse de swing-up ({w_swing:.1f} rad/s)."
            )
        ql_slew = float(getattr(
            cfg, "QL_DUTY_SLEW_PER_S", cfg.DUTY_SLEW_PER_S))
        reversal = 2.0 * cfg.QL_U_MAX / ql_slew
        if reversal > cfg.RL_DT:
            out.append(
                f"QL_DUTY_SLEW_PER_S = {ql_slew:.0f}/s : une inversion "
                f"pleine échelle prend {1000 * reversal:.0f} ms alors qu'un pas RL "
                f"dure {1000 * cfg.RL_DT:.0f} ms — le couple demandé n'est jamais "
                f"atteint."
            )
        if cfg.QL_TDOT_MAX > 0.0 and cfg.THETA_DOT_MAX <= cfg.QL_TDOT_MAX:
            out.append(
                f"THETA_DOT_MAX ({cfg.THETA_DOT_MAX:.0f}) <= QL_TDOT_MAX "
                f"({cfg.QL_TDOT_MAX:.0f}) : la sécurité globale coupe le mode "
                f"AVANT le terminal d'épisode, l'agent tombe en faute au lieu "
                f"d'apprendre de la pénalité."
            )
        if cfg.QL_U_MAX > cfg.DUTY_LIMIT:
            out.append(
                f"QL_U_MAX ({cfg.QL_U_MAX}) > DUTY_LIMIT ({cfg.DUTY_LIMIT}) : "
                f"les actions extrêmes sont écrêtées par Motor::setDuty."
            )
        if self.breakaway_duty >= cfg.QL_U_MIN:
            out.append(
                f"QL_U_MIN ({cfg.QL_U_MIN}) <= duty de décollement mesurée "
                f"({self.breakaway_duty}) : les plus petites actions du RL ne "
                f"produisent toujours AUCUN mouvement."
            )
        if cfg.SWING_KICK_U <= self.breakaway_duty:
            out.append(
                f"SWING_KICK_U ({cfg.SWING_KICK_U}) <= duty de décollement "
                f"({self.breakaway_duty}) : l'impulsion d'amorçage du mode "
                f"classique ne décolle pas le bras, le swing-up ne démarre jamais."
            )
        return out

    @classmethod
    def from_overrides(cls, pairs: dict[str, str]) -> "Rig":
        rig = cls()
        known = {f.name: f.type for f in fields(cls)}
        for key, raw in pairs.items():
            if key not in known:
                raise SystemExit(
                    f"paramètre rig inconnu : {key}\n"
                    f"connus : {', '.join(sorted(known))}"
                )
            cur = getattr(rig, key)
            if isinstance(cur, bool):
                setattr(rig, key, str(raw).lower() in ("1", "true", "oui", "yes"))
            elif isinstance(cur, str):
                setattr(rig, key, raw)
            else:
                setattr(rig, key, float(raw))
        return rig


def load(config_overrides: dict[str, str] | None = None,
         rig_overrides: dict[str, str] | None = None) -> tuple[Cfg, Rig]:
    cfg = parse_config_h()
    for key, raw in (config_overrides or {}).items():
        if key not in cfg:
            raise SystemExit(f"constante inconnue dans config.h : {key}")
        cfg[key] = type(cfg[key])(float(raw))
    # Les constantes dérivées de config.h doivent suivre les surcharges.
    cfg["RL_DT"] = cfg["CTRL_DT"] * cfg["RL_DIVIDER"]
    cfg.sync()          # après TOUTES les écritures, cf. Cfg.sync()
    return cfg, Rig.from_overrides(rig_overrides or {})
