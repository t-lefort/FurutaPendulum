"""
Tile coding : approximation linéaire à traits binaires, en remplacement de la
Q-table.

POURQUOI. Une table traite chaque case comme indépendante : ce qui est appris à
alpha = 30,0 deg n'apprend RIEN à 30,2 deg. Conséquence mesurée en simulation :
chaque ajout de résolution (dimension theta_dot, actions plus fines) a RALENTI
l'apprentissage au lieu de l'améliorer, parce qu'il multipliait le nombre de
cases à visiter individuellement.

COMMENT. On empile `n_tilings` quadrillages, chacun décalé d'une fraction de
case. Un état active exactement un pavé par quadrillage. La valeur est la SOMME
des poids des pavés actifs, et une mise à jour écrit dans chacun (divisée par le
nombre de quadrillages). Deux états voisins partagent la plupart de leurs pavés,
donc apprendre sur l'un généralise à l'autre : c'est ce qui permet d'avoir à la
fois une résolution fine et un nombre d'échantillons raisonnable.

Décalages ASYMÉTRIQUES (1, 3, 5, ... par dimension) : c'est la recommandation
standard, les décalages uniformes créent des artefacts d'alignement où plusieurs
quadrillages basculent au même endroit.

`alpha` est CIRCULAIRE (wrap=True) : -pi et +pi sont le même angle physique, un
pavé doit enjamber la coupure. Les vitesses sont bornées (wrap=False) et
saturent aux extrémités, comme dans le firmware.
"""

from __future__ import annotations

import math
import random
from array import array

from .agent import QLearning
from .fw_config import Cfg

_TWO_PI = 2.0 * math.pi


def _mix32(x: int) -> int:
    """Mélangeur entier 32 bits (style Murmur/Knuth). Déterministe et trivial à
    réécrire à l'identique en C++ — indispensable pour que la table entraînée en
    simulation reste valide sur la Teensy."""
    x &= 0xFFFFFFFF
    x = (x * 0x9E3779B1) & 0xFFFFFFFF
    x ^= x >> 15
    x = (x * 0x85EBCA77) & 0xFFFFFFFF
    x ^= x >> 13
    return x


class TileCoder:
    def __init__(self, dims, n_tilings: int, hash_bits: int = 0):
        """dims : liste de (lo, hi, n_tiles, wrap).

        hash_bits > 0 : les indices de pavés sont HACHÉS dans une table de
        2^hash_bits traits, au lieu d'être indexés directement. C'est la forme
        classique du tile coding, et ici c'est la condition pour tenir sur la
        Teensy : l'indexation directe demande 4,64 Mo pour 512 ko de DMAMEM.
        Les collisions sont bénignes tant que la table reste grande devant le
        nombre d'états RÉELLEMENT visités — le pendule ne parcourt qu'une petite
        variété de l'espace d'état, pas tout le produit cartésien.
        """
        self.dims = dims
        self.T = int(n_tilings)
        self.hash_bits = int(hash_bits)
        self.mask = (1 << self.hash_bits) - 1 if self.hash_bits > 0 else 0
        # Une dimension non circulaire a besoin d'une case de plus : le décalage
        # des quadrillages fait dépasser d'un pavé au-delà de la borne.
        self.shape = [n if wrap else n + 1 for (_, _, n, wrap) in dims]
        self.per_tiling = 1
        for s in self.shape:
            self.per_tiling *= s
        self.n_feat = (1 << self.hash_bits) if self.hash_bits > 0 \
            else self.T * self.per_tiling

        self.width = [(hi - lo) / n for (lo, hi, n, _) in dims]
        self.offsets = []
        for t in range(self.T):
            self.offsets.append([
                self.width[i] * (((2 * i + 1) * t) % self.T) / self.T
                for i in range(len(dims))
            ])

    def active(self, x) -> list[int]:
        """Indices des pavés actifs : un par quadrillage."""
        feats = []
        for t in range(self.T):
            off = self.offsets[t]
            idx = 0
            for i, (lo, hi, n, wrap) in enumerate(self.dims):
                c = int(math.floor((x[i] + off[i] - lo) / self.width[i]))
                if wrap:
                    c %= n
                elif c < 0:
                    c = 0
                elif c > n:
                    c = n
                idx = idx * self.shape[i] + c
            if self.hash_bits > 0:
                feats.append(_mix32(t * 0x27D4EB2D + idx) & self.mask)
            else:
                feats.append(t * self.per_tiling + idx)
        return feats


class TileQLearning(QLearning):
    """Même logique de contrôle que QLearning (récompense, terminaux, pause,
    exploration persistante, shaping potentiel, traces) mais la fonction de
    valeur est un tile coder au lieu d'une table.

    On hérite volontairement de QLearning pour que reward()/potential() et les
    règles terminales restent PARTAGÉES : deux copies divergeraient et la simu
    ne validerait plus le même agent.
    """

    def __init__(self, cfg: Cfg, rng: random.Random | None = None):
        super().__init__(cfg, rng)

        n_alpha = int(getattr(cfg, "TC_N_ALPHA", 12))
        n_adot = int(getattr(cfg, "TC_N_ADOT", 12))
        n_tdot = int(getattr(cfg, "TC_N_TDOT", 0))
        self.alpha_warp = float(getattr(cfg, "TC_ALPHA_WARP", 1.0))
        self.T = int(getattr(cfg, "TC_TILINGS", 8))

        adot = float(cfg.QL_ADOT_MAX)
        dims = [(-math.pi, math.pi, n_alpha, True),
                (-adot, adot, n_adot, False)]
        self.use_tdot = n_tdot > 0
        if self.use_tdot:
            tdot = float(getattr(cfg, "QL_TDOT_BIN_MAX", 12.0))
            dims.append((-tdot, tdot, n_tdot, False))

        self.coder = TileCoder(dims, self.T,
                               int(getattr(cfg, "TC_HASH_BITS", 0)))
        # Poids : un par (pavé, action). Remplace self.Q.
        self.W = array("f", bytes(4 * self.coder.n_feat * self.n_act))
        # Le pas d'apprentissage se divise par le nombre de quadrillages, sinon
        # la mise à jour effective est T fois trop grande (T pavés contribuent).
        self.lr = self.learning_rate / float(self.T)
        self.Etile: dict[int, float] = {}
        self.prev_feats: list[int] | None = None

    # ---- Fonction de valeur ----
    def q(self, feats, a: int) -> float:
        W, na = self.W, self.n_act
        return sum(W[f * na + a] for f in feats)

    def eval_all(self, feats) -> tuple[int, float]:
        """(meilleure action, valeur max) en UN seul balayage.

        Un appel coûte n_act * n_tilings lectures. Séparer best_action et max_q,
        et rappeler best_action pour le test de Watkins, faisait 3 balayages par
        pas RL au lieu d'un — c'était le goulot de la simulation (~170 lectures
        par pas contre 7 pour une table).
        """
        W, na = self.W, self.n_act
        best = self.act_neutral            # égalité tranchée vers le couple nul
        bv = sum(W[f * na + best] for f in feats)
        for a in range(na):
            if a == best:
                continue
            v = sum(W[f * na + a] for f in feats)
            if v > bv:
                bv, best = v, a
        return best, bv

    def _update(self, feats, a: int, delta: float, terminal: bool) -> None:
        W, na, lr = self.W, self.n_act, self.lr
        if self.lam <= 0.0:
            for f in feats:
                W[f * na + a] += lr * delta
            return
        # Traces remplaçantes sur les traits actifs.
        for f in feats:
            for b in range(na):
                self.Etile.pop(f * na + b, None)
            self.Etile[f * na + a] = 1.0
        decay = self.cfg.QL_GAMMA * self.lam
        cut = self.trace_min
        dead = []
        for k, e in self.Etile.items():
            W[k] += lr * delta * e
            e *= decay
            if e < cut:
                dead.append(k)
            else:
                self.Etile[k] = e
        for k in dead:
            del self.Etile[k]
        # Plafond DUR sur le nombre de traces. Le dict garde l'ordre d'insertion
        # et une trace décroît de façon monotone avec son âge : les plus
        # anciennes sont donc les plus faibles, on les jette en premier.
        # Nécessaire pour la Teensy (tampon borné) et décisif pour la vitesse de
        # simulation : a lambda=0.92 et 200 Hz, l'historique fait 109 pas, soit
        # ~870 traces actives parcourues à CHAQUE pas RL.
        while len(self.Etile) > self.trace_max:
            del self.Etile[next(iter(self.Etile))]
        if terminal:
            self.Etile.clear()

    def _select_action_f(self, feats, alpha: float, alpha_dot: float) -> int:
        """Pendant de QLearning._select_action, sur les traits du tile coder."""
        kick = self.kick_action(alpha, alpha_dot)
        if kick is not None:
            return kick
        if self.greedy:
            return self.eval_all(feats)[0]
        near = float(getattr(self.cfg, "QL_EXPLORE_NEAR_RAD", 0.0))
        is_near = near > 0.0 and abs(alpha) < near
        if self.explore_hold > 0:
            if is_near:
                self.explore_hold = 0
            else:
                self.explore_hold -= 1
                return self.explore_act
        eps = self.exploration_epsilon(is_near)
        if self.rng.randrange(10000) / 10000.0 < eps:
            self.explore_act = self.rng.randrange(self.n_act)
            self.explore_hold = self.explore_hold_for(alpha) - 1
            return self.explore_act
        return self.eval_all(feats)[0]

    def warp_alpha(self, a: float) -> float:
        """Déformation de l'axe alpha : dense près du haut, grossier en bas.

        Le pavage est uniforme dans la coordonnée déformée, donc étirer la zone
        proche de zéro y met MÉCANIQUEMENT plus de pavés. Motivation mesurée :
        l'équilibre exige ~0,5 deg de résolution, mais seulement sur +/-10 deg ;
        le swing-up se contente de quelques degrés sur 360. Une grille uniforme
        paie donc la finesse partout, ce qui gonfle le nombre d'indices distincts
        (173 600) et fait exploser les collisions du hachage — mesuré : à 2^13 le
        swing-up tombe à 0 % alors que l'équilibre, lui, va très bien (3,10 s).

        u = signe(a) * pi * (|a|/pi)^p, avec p < 1. Continue en +/-pi, donc la
        circularité de alpha est préservée.
        """
        p = self.alpha_warp
        if p == 1.0:
            return a
        s = -1.0 if a < 0.0 else 1.0
        return s * math.pi * (abs(a) / math.pi) ** p

    def _feats(self, alpha, alpha_dot, theta_dot):
        a = self.warp_alpha(alpha)
        x = [a, alpha_dot, theta_dot] if self.use_tdot else [a, alpha_dot]
        return self.coder.active(x)

    def end_episode(self) -> None:
        super().end_episode()
        self.lr = self.learning_rate / float(self.T)
        self.Etile.clear()
        self.prev_feats = None

    # ---- Un pas RL ----
    def step(self, theta, alpha, theta_dot, alpha_dot, on_rezero):
        cfg = self.cfg
        new_episode = False

        if self.paused:
            self.pause_time += cfg.RL_DT
            self.u_command = 0.0
            self.last_action = 0
            settled = (abs(alpha) > math.pi - cfg.QL_SETTLE_RAD
                       and abs(alpha_dot) < cfg.QL_SETTLE_ADOT
                       and abs(theta_dot) < cfg.QL_SETTLE_TDOT)
            if not settled and self.pause_time <= cfg.QL_SETTLE_MAX_S:
                return 0.0, False
            on_rezero()
            self.paused = False
            self.pause_time = 0.0
            self.prev_feats = None
            self.steps_in_episode = 0.0
            self.episode_reward = 0.0

        feats = self._feats(alpha, alpha_dot, theta_dot)
        self.note_first_up(alpha)
        out_of_range, term_pen = self.terminal_check(theta, theta_dot, alpha)
        phi = (self.potential(alpha, alpha_dot)
               + self.approach_potential(alpha, alpha_dot, theta_dot))

        # SARSA(lambda) : l'action du nouvel état doit être choisie AVANT la mise
        # à jour, puisque le bootstrap se fait sur Q(s', a').
        a_next = None
        if self.sarsa and not out_of_range:
            a_next = self._select_action_f(feats, alpha, alpha_dot)

        if not self.greedy and self.prev_feats is not None:
            r = self.reward(alpha, alpha_dot, theta_dot, self.prev_action)
            r += (0.0 if out_of_range else cfg.QL_GAMMA * phi) - self.prev_phi
            if out_of_range:
                r += term_pen
            self.last_step_reward = r
            self.episode_reward += r
            if out_of_range:
                target = r
            elif self.sarsa:
                target = r + cfg.QL_GAMMA * self.q(feats, a_next)
            else:
                _, mx = self.eval_all(feats)
                target = r + cfg.QL_GAMMA * mx
            delta = target - self.q(self.prev_feats, self.prev_action)
            self._update(self.prev_feats, self.prev_action, delta, out_of_range)

        if out_of_range:
            self.end_episode()
            self._begin_pause()
            self.u_command = 0.0
            return 0.0, True

        a = a_next if a_next is not None else self._select_action_f(feats, alpha, alpha_dot)

        # Watkins Q(lambda) coupe les traces après une action exploratoire.
        # SARSA(lambda) ne coupe PAS : il apprend la valeur de la politique
        # suivie, exploration comprise — couper le ramènerait au hors-politique.
        if self.lam > 0.0 and not self.greedy and not self.sarsa:
            greedy_a, _ = self.eval_all(feats)
            if a != greedy_a:
                self.Etile.clear()

        self.prev_feats = feats
        self.prev_action = a
        self.prev_phi = phi
        self.last_action = a - self.act_neutral
        self.u_command = self.action_u(a)

        self.steps_in_episode += 1.0
        if self.steps_in_episode * cfg.RL_DT >= self.episode_limit_s:
            self.end_episode()
            new_episode = True
            self._begin_pause()
        return self.u_command, new_episode

    # ---- Persistance ----
    # Format PROPRE au tile coding : incompatible avec storage.cpp, qui attend
    # une table dense. Le portage firmware demanderait un nouveau QHeader.
    def save_bin(self, path: str) -> None:
        import struct
        from pathlib import Path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            f.write(struct.pack("<IHHHH", 0x54494C31, 1, self.T,
                                self.coder.per_tiling, self.n_act))
            f.write(self.W.tobytes())

    def load_bin(self, path: str) -> bool:
        import struct
        with open(path, "rb") as f:
            magic, ver, T, per, na = struct.unpack("<IHHHH", f.read(12))
            if (magic != 0x54494C31 or T != self.T
                    or per != self.coder.per_tiling or na != self.n_act):
                return False
            data = f.read()
        if len(data) != len(self.W) * 4:
            return False
        self.W = array("f")
        self.W.frombytes(data)
        return True


class SplitTileQLearning(TileQLearning):
    """Tile coding factorisé sous le budget RAM2 du Teensy 4.1.

    Ce n'est pas un contrôleur hybride : il n'y a qu'une politique SARSA et une
    seule fonction Q. Seule la représentation change selon la région d'état :
    une banque globale 3D apprend le swing-up, tandis qu'une banque locale 3D
    plus fine représente le voisinage du sommet. En mode gate dur, les deux
    banques ne sont jamais actives ensemble ; le mode overlap conserve aussi
    les traits globaux près du sommet et fait apprendre un résidu à la banque
    locale.
    """

    def __init__(self, cfg: Cfg, rng: random.Random | None = None):
        # Initialise le cycle de vie SARSA sans allouer la grande table standard.
        QLearning.__init__(self, cfg, rng)

        self.local_rad = float(cfg.TC_LOCAL_RAD)
        self.global_T = int(cfg.TC_GLOBAL_TILINGS)
        self.local_T = int(cfg.TC_LOCAL_TILINGS)
        self.T = self.global_T  # compatibilité d'introspection/persistance

        adot = float(cfg.QL_ADOT_MAX)
        tdot = float(cfg.QL_TDOT_BIN_MAX)
        self.global_coder = TileCoder(
            [(-math.pi, math.pi, int(cfg.TC_GLOBAL_N_ALPHA), True),
             (-adot, adot, int(cfg.TC_GLOBAL_N_ADOT), False),
             (-tdot, tdot, int(cfg.TC_GLOBAL_N_TDOT), False)],
            self.global_T,
        )

        local_adot = float(cfg.TC_LOCAL_ADOT_MAX)
        local_tdot = float(getattr(cfg, "TC_LOCAL_TDOT_MAX", tdot))
        self.local_coder = TileCoder(
            [(-self.local_rad, self.local_rad, int(cfg.TC_LOCAL_N_ALPHA), False),
             (-local_adot, local_adot, int(cfg.TC_LOCAL_N_ADOT), False),
             (-local_tdot, local_tdot, int(cfg.TC_LOCAL_N_TDOT), False)],
            self.local_T,
        )
        self.local_offset = self.global_coder.n_feat
        self.n_features = self.local_offset + self.local_coder.n_feat
        self.W = array("f", bytes(4 * self.n_features * self.n_act))
        self.lr = self.learning_rate  # normalisé par banque dans _update()
        self.Etile: dict[int, float] = {}
        self.prev_feats: list[int] | None = None

    @property
    def memory_bytes(self) -> int:
        return len(self.W) * self.W.itemsize

    def _feats(self, alpha, alpha_dot, theta_dot):
        if abs(alpha) < self.local_rad:
            local = [self.local_offset + f for f in self.local_coder.active(
                [alpha, alpha_dot, theta_dot])]
            if int(getattr(self.cfg, "TC_SPLIT_OVERLAP", 0)) > 0:
                return self.global_coder.active(
                    [alpha, alpha_dot, theta_dot]) + local
            return local
        return self.global_coder.active([alpha, alpha_dot, theta_dot])

    def _update(self, feats, a: int, delta: float, terminal: bool) -> None:
        """Même mise à jour que TileQLearning, avec 4 ou 8 traits actifs.

        Le pas doit être divisé par le nombre de pavages de la banque courante ;
        utiliser une constante unique donnerait deux fois plus de gain au global.
        """
        W, na = self.W, self.n_act
        base_lr = self.learning_rate

        def rate(feature: int) -> float:
            if feature >= self.local_offset:
                return (base_lr * float(self.cfg.TC_LOCAL_LR_SCALE)
                        / float(self.local_T))
            return (base_lr * float(getattr(
                self.cfg, "TC_GLOBAL_LR_SCALE", 1.0))
                    / float(self.global_T))

        if self.lam <= 0.0:
            for f in feats:
                W[f * na + a] += rate(f) * delta
            return
        for f in feats:
            for b in range(na):
                self.Etile.pop(f * na + b, None)
            self.Etile[f * na + a] = 1.0
        decay = self.cfg.QL_GAMMA * self.lam
        dead = []
        for k, e in self.Etile.items():
            W[k] += rate(k // na) * delta * e
            e *= decay
            if e < self.trace_min:
                dead.append(k)
            else:
                self.Etile[k] = e
        for k in dead:
            del self.Etile[k]
        while len(self.Etile) > self.trace_max:
            del self.Etile[next(iter(self.Etile))]
        if terminal:
            self.Etile.clear()

    def save_bin(self, path: str) -> None:
        import struct
        from pathlib import Path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "wb") as f:
            # 'SPL1', version, Tg, Tl, global/per, local/per, actions.
            f.write(struct.pack("<IHHHIIH", 0x53504C31, 1,
                                self.global_T, self.local_T,
                                self.global_coder.per_tiling,
                                self.local_coder.per_tiling, self.n_act))
            f.write(self.W.tobytes())

    def load_bin(self, path: str) -> bool:
        import struct
        header = "<IHHHIIH"
        with open(path, "rb") as f:
            raw = f.read(struct.calcsize(header))
            magic, ver, tg, tl, gp, lp, na = struct.unpack(header, raw)
            if (magic != 0x53504C31 or ver != 1
                    or tg != self.global_T or tl != self.local_T
                    or gp != self.global_coder.per_tiling
                    or lp != self.local_coder.per_tiling or na != self.n_act):
                return False
            data = f.read()
        if len(data) != len(self.W) * self.W.itemsize:
            return False
        self.W = array("f")
        self.W.frombytes(data)
        return True
