"""
Port fidèle de qlearning.cpp.

Toute divergence avec le C++ rend la simulation inutile : c'est CET agent qu'on
valide, pas une réimplémentation « propre ». Chaque fonction ci-dessous suit
ligne à ligne son homologue firmware — y compris les détails qui comptent :
départage des égalités vers l'action neutre, absence de bootstrap sur l'état
terminal, saut du test de sortie de plage au premier pas, exploration tenue.

La table est sérialisable au format de storage.cpp (/q_current.bin), donc une
politique entraînée ici part telle quelle sur la carte SD de la Teensy.
"""

from __future__ import annotations

import math
import random
import struct
from array import array
from pathlib import Path

from .fw_config import Cfg


class QLearning:
    def __init__(self, cfg: Cfg, rng: random.Random | None = None):
        self.cfg = cfg
        self.rng = rng or random.Random(0)

        self.n_alpha = int(cfg.QL_N_ALPHA)
        self.n_adot = int(cfg.QL_N_ADOT)
        # 3e dimension d'etat OPTIONNELLE : theta_dot. A 1 (defaut) l'etat reste
        # [alpha, alpha_dot] et le comportement est identique au firmware actuel.
        # Au-dela, l'agent OBSERVE la vitesse du bras — sans elle il ne peut pas
        # representer un rattrapage : la loi d'equilibre classique a besoin du
        # terme K_THD*theta_dot (verifie en simu : a K_THD = 0 l'equilibre tombe).
        self.n_tdot = max(1, int(getattr(cfg, "QL_N_TDOT", 1)))
        self.tdot_max = float(getattr(cfg, "QL_TDOT_BIN_MAX", 12.0))
        self.n_act = int(cfg.QL_N_ACT)
        self.act_neutral = self.n_act // 2
        self.n_lvl = self.act_neutral

        n = self.n_alpha * self.n_adot * self.n_tdot * self.n_act
        self.Q = array("f", bytes(4 * n))
        # Traces d'éligibilité (Q(lambda)). lambda = 0 -> pas de traces, pas de
        # tableau alloué : comportement Q-learning à un pas, identique à avant.
        self.lam = float(getattr(cfg, "QL_LAMBDA", 0.0))
        # Traces CREUSES : seules les paires (etat, action) récemment visitées
        # ont une trace non négligeable. Avec gamma*lambda ~ 0,6 et un seuil de
        # 1e-4, l'historique utile fait ~18 pas, contre 14 000 cases à balayer
        # pour un tableau plein — indispensable en Python, et de toute façon la
        # bonne implémentation sur Teensy.
        self.E: dict[int, float] = {}
        # Coupure et plafond des traces. Le seuil était figé à 1e-4 : on traînait
        # des traces valant 0,0001, sans effet mesurable et payées plein tarif.
        self.trace_min = float(getattr(cfg, "QL_TRACE_MIN", 1e-2))
        self.trace_max = int(getattr(cfg, "QL_TRACE_MAX", 512))
        self.prev_phi = 0.0
        self.kick_time = 0.0
        # None = pas de terminaison sur chute (épisode de swing-up normal).
        self.fall_limit: float | None = None
        # SARSA(lambda) au lieu de Q(lambda) : bootstrap sur l'action RÉELLEMENT
        # choisie. Près d'un équilibre instable, max_a Q(s',a) est la valeur
        # d'une politique gloutonne qui tiendrait indéfiniment, alors que le
        # comportement réel (epsilon-glouton) fait tomber le pendule — la cible
        # est donc systématiquement optimiste et l'agent n'apprend jamais que SA
        # conduite perd. C'est le cas d'école "cliff walking" où SARSA gagne.
        self.sarsa = float(getattr(cfg, "QL_SARSA", 0.0)) > 0.0

        self.greedy = False
        self.prev_state_idx = -1
        self.prev_action = 0
        self.steps_in_episode = 0.0
        self.episode_limit_s = float(cfg.QL_EPISODE_S)
        self.first_up_seen = False
        self.after_up_armed = False
        self.up_hold_time = 0.0
        self.paused = False
        self.pause_time = 0.0
        self.explore_hold = 0
        self.explore_act = 0

        # Stats (QLearning::Stats)
        self.episode = 0
        self.epsilon = cfg.QL_EPS0
        self.epsilon_top = float(getattr(cfg, "QL_EPS_TOP0", -1.0))
        self.top_explore_steps = 0
        self.learning_rate = float(cfg.QL_LR)
        self.episode_reward = 0.0
        self.best_reward = -1e9
        self.last_action = 0
        self.u_command = 0.0
        self.last_step_reward = 0.0
        self.last_episode_reward = 0.0

    # ---- Jeu d'actions : niveaux répartis sur [uMin, uMax] ----
    def action_u(self, a: int) -> float:
        k = a - self.act_neutral
        if k == 0:
            return 0.0
        u_min = self.cfg.QL_U_MIN
        u_max = max(self.cfg.QL_U_MAX, u_min)
        t = (abs(k) - 1) / (self.n_lvl - 1) if self.n_lvl > 1 else 1.0
        u = u_min + t * (u_max - u_min)
        return u if k > 0 else -u

    # ---- Discrétisation ----
    def bin_alpha(self, a: float) -> int:
        b = int((a + math.pi) / (2.0 * math.pi) * self.n_alpha)
        return max(0, min(self.n_alpha - 1, b))

    def bin_adot(self, w: float) -> int:
        lim = self.cfg.QL_ADOT_MAX
        w = max(-lim, min(lim, w))
        b = int((w + lim) / (2.0 * lim) * self.n_adot)
        return max(0, min(self.n_adot - 1, b))

    def bin_tdot(self, w: float) -> int:
        if self.n_tdot <= 1:
            return 0
        lim = self.tdot_max
        w = max(-lim, min(lim, w))
        b = int((w + lim) / (2.0 * lim) * self.n_tdot)
        return max(0, min(self.n_tdot - 1, b))

    def state_index(self, alpha: float, alpha_dot: float,
                    theta_dot: float = 0.0) -> int:
        i = self.bin_alpha(alpha) * self.n_adot + self.bin_adot(alpha_dot)
        return (i * self.n_tdot + self.bin_tdot(theta_dot)) * self.n_act

    def best_action(self, s_idx: int) -> int:
        Q = self.Q
        best = self.act_neutral
        bv = Q[s_idx + best]
        for a in range(self.n_act):
            v = Q[s_idx + a]
            if v > bv:
                bv = v
                best = a
        return best

    def _td_traces(self, i: int, delta: float, s_idx: int, terminal: bool) -> None:
        """Mise à jour Q(lambda) avec traces REMPLAÇANTES.

        Sans traces, la valeur ne remonte que d'un état par visite : c'est ce qui
        explique le plateau observé (récompense figée après ~200 épisodes). Avec
        traces, le crédit est propagé immédiatement à toute la trajectoire
        récente. lambda utile : 0,4 à 0,8 d'après la littérature.
        """
        cfg = self.cfg
        Q, E = self.Q, self.E
        # Trace remplaçante : 1 sur (s,a), et on efface les AUTRES actions du
        # même état (sinon deux actions du même état restent créditées).
        base = i - (i % self.n_act)
        for b in range(self.n_act):
            E.pop(base + b, None)
        E[i] = 1.0
        lr, decay = self.learning_rate, cfg.QL_GAMMA * self.lam
        cut = self.trace_min
        dead = []
        for k, e in E.items():
            Q[k] += lr * delta * e
            e *= decay
            if e < cut:
                dead.append(k)
            else:
                E[k] = e
        for k in dead:
            del E[k]
        # Plafond dur : le dict garde l'ordre d'insertion et une trace décroît
        # avec son âge, donc les plus anciennes sont les plus faibles.
        while len(E) > self.trace_max:
            del E[next(iter(E))]
        if terminal:
            E.clear()

    def max_q(self, s_idx: int) -> float:
        Q = self.Q
        bv = Q[s_idx]
        for a in range(1, self.n_act):
            v = Q[s_idx + a]
            if v > bv:
                bv = v
        return bv

    # ---- Récompense ----
    def reward(self, alpha, alpha_dot, theta_dot, action) -> float:
        cfg = self.cfg
        r = 1.0 + math.cos(alpha)
        if abs(alpha) < cfg.QL_ADOT_TOP_RAD:
            r -= cfg.QL_K_ADOT_TOP * abs(alpha_dot)
        if abs(alpha) < float(getattr(cfg, "QL_TDOT_TOP_RAD", cfg.QL_ADOT_TOP_RAD)):
            r -= float(getattr(cfg, "QL_K_TDOT_TOP", 0.0)) * theta_dot * theta_dot
        # Cône de récompense près du haut : c'est LE gradient qui manquait.
        # 1+cos(alpha) est un extremum en 0, donc plat au second ordre : sur
        # +/-8 deg il ne varie que de 0,011 pour une base de 2 (0,5 %), noyé sous
        # le coût d'effort (0,02). L'agent n'avait aucun signal lui indiquant
        # quelle action GARDE le pendule en haut — mesuré : 0,10 s de tenue même
        # avec 70 % des épisodes dédiés à l'équilibre, alors que la même loi
        # classique quantifiée sur les mêmes 7 actions tient 29,4 s.
        # Forme en cône (max au sommet, nul au bord) : reste NON NÉGATIF, donc
        # l'invariant "pas de puits négatif" tient.
        if cfg.QL_K_BAL > 0.0:
            ar = alpha / cfg.QL_BAL_CONE_RAD
            wr = alpha_dot / max(cfg.QL_BAL_CONE_ADOT, 1e-6)
            cone = max(0.0, 1.0 - ar * ar - wr * wr)
            # Freinage progressif du bras : à angle/vitesse pendule identiques,
            # une arrivée avec theta_dot faible doit être plus désirable. Un
            # simple gate dans QL_R_UP ne fournit aucun gradient avant le seuil.
            td_scale = float(getattr(cfg, "QL_BAL_CONE_TDOT", 0.0))
            arm_taper = (max(0.0, 1.0 - (theta_dot / td_scale) ** 2)
                         if td_scale > 0.0 else 1.0)
            r += cfg.QL_K_BAL * cone * arm_taper
        td_over = abs(theta_dot) - cfg.QL_TDOT_SOFT
        if td_over > 0.0:
            r -= cfg.QL_K_TDOT * td_over
        r -= 0.02 * abs(self.action_u(action)) / max(cfg.QL_U_MAX, 1e-3)
        if abs(alpha) < cfg.QL_UP_RAD and abs(theta_dot) < cfg.QL_UP_TDOT:
            r += cfg.QL_R_UP
        if (abs(alpha) < cfg.QL_BAL_RAD and abs(alpha_dot) < cfg.QL_BAL_ADOT
                and abs(theta_dot) < cfg.QL_BAL_TDOT):
            r += cfg.QL_R_BAL
        return r

    def _select_action(self, s_idx: int, alpha: float, alpha_dot: float) -> int:
        """Choix de l'action : amorçage, puis glouton / rafale / epsilon.

        Extrait de step() parce que SARSA doit choisir l'action du NOUVEL état
        avant de mettre à jour la transition précédente.
        """
        kick = self.kick_action(alpha, alpha_dot)
        if kick is not None:
            return kick
        if self.greedy:
            return self.best_action(s_idx)
        near = float(getattr(self.cfg, "QL_EXPLORE_NEAR_RAD", 0.0))
        is_near = near > 0.0 and abs(alpha) < near
        if self.explore_hold > 0:
            # Une rafale tirée pour le swing-up ne doit pas traverser la
            # frontière et imposer encore 100 ms de couple aléatoire au
            # rattrapage. Près du haut on reprend une décision immédiatement.
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
        return self.best_action(s_idx)

    def exploration_epsilon(self, is_near: bool) -> float:
        """Epsilon global, ou epsilon local décrémenté par VISITE du sommet."""
        if not is_near:
            return self.epsilon
        top0 = float(getattr(self.cfg, "QL_EPS_TOP0", -1.0))
        if top0 >= 0.0:
            eps = self.epsilon_top
            if not self.greedy:
                self.top_explore_steps += 1
                self.epsilon_top = max(
                    float(getattr(self.cfg, "QL_EPS_TOP_MIN", 0.0)),
                    self.epsilon_top * float(getattr(
                        self.cfg, "QL_EPS_TOP_DECAY", 1.0)))
            return eps
        top_cap = float(getattr(self.cfg, "QL_EXPLORE_EPS_TOP", -1.0))
        return min(self.epsilon, top_cap) if top_cap >= 0.0 else self.epsilon

    def explore_hold_for(self, alpha: float) -> int:
        """Durée d'une rafale d'exploration, SELON L'ÉTAT.

        Conflit mesuré : le pompage a besoin de poussées soutenues (sans rafales
        longues le swing-up ne s'apprend pas du tout — l'exploration à 20 Hz
        redevient du bruit blanc incapable d'exciter un oscillateur à 1,5 Hz),
        alors que le rattrapage les interdit (une action aléatoire tenue 200 ms
        fait tomber le pendule, qui bascule en ~100 ms).
        Mesuré à 200 Hz : rafales de 200 ms -> tenue 0,17 s ; 40 ms -> 0,45 s ;
        5 ms -> 0,02 s mais swing-up détruit (0 %).
        Un réglage unique ne peut donc que faire un compromis. En rendant la
        durée dépendante de |alpha|, on garde les deux : long en bas pour pomper,
        court près du haut pour ne pas saboter l'équilibre.
        """
        cfg = self.cfg
        near = getattr(cfg, "QL_EXPLORE_NEAR_RAD", 0.0)
        if near > 0.0 and abs(alpha) < near:
            return max(1, int(getattr(cfg, "QL_EXPLORE_HOLD_TOP", 1)))
        return max(1, int(cfg.QL_EXPLORE_HOLD))

    def terminal_check(self, theta, theta_dot, alpha) -> tuple[bool, float]:
        """(terminal, pénalité). Partagé par l'agent table et l'agent tile.

        `fall_limit` est posé par le Runner au début d'un épisode d'ÉQUILIBRE
        (départ près du haut) : dès que le pendule décroche, l'épisode s'arrête.
        Sans ça, un épisode d'équilibre qui tombe redevient un épisode de
        swing-up ordinaire et gaspille le reste des 15 s — c'est justement ce qui
        affame la zone d'équilibre en échantillons.

        /!\\ La chute n'a PAS de pénalité explicite. Perdre le bootstrap suffit :
        près du haut la valeur vaut plusieurs centaines, la récompense
        immédiate ~2, donc terminer coûte déjà très cher. Ajouter QL_R_OUT_RANGE
        risquerait de recréer la pathologie déjà rencontrée où se suicider
        rapporte plus que survivre.
        """
        cfg = self.cfg
        if self.steps_in_episode <= 0.0:
            return False, 0.0
        after_up_fall = float(getattr(cfg, "QL_AFTER_UP_FALL_RAD", 0.0))
        if (after_up_fall > 0.0 and self.after_up_armed
                and abs(alpha) > after_up_fall):
            # La perte du bootstrap suffit comme signal d'echec : pas de
            # penalite arbitraire et pas d'etat artificiel.
            return True, 0.0
        if cfg.QL_THETA_TURNS > 0.0 and abs(theta) > cfg.QL_THETA_TURNS * 2.0 * math.pi:
            return True, cfg.QL_R_OUT_RANGE
        if cfg.QL_TDOT_MAX > 0.0 and abs(theta_dot) > cfg.QL_TDOT_MAX:
            return True, cfg.QL_R_OUT_RANGE
        if self.fall_limit is not None and abs(alpha) > self.fall_limit:
            return True, 0.0
        return False, 0.0

    def kick_action(self, alpha: float, alpha_dot: float) -> int | None:
        """Amorçage, même rôle que SWING_KICK_* dans control_classic.cpp.

        Au repos exact en bas, l'agent a APPRIS que l'action neutre est la
        meilleure (mesuré : ligne Q = [1.3, 0.7, 0.9, **10.4**, 1.2, 0.9, 3.1]),
        et il y reste indéfiniment — c'est le "pendule qui ne bouge pas" observé
        en glouton. Deux raisons, les deux réelles :
          - rien ne casse la symétrie : en entraînement c'est epsilon qui le
            faisait, en glouton plus rien ;
          - avec le shaping potentiel, rester en bas rapporte (gamma-1)*Phi > 0
            par pas, soit un vrai bonus à l'immobilité.
        Interdire l'immobilité DANS cet état supprime l'optimum au lieu d'essayer
        de le contrebalancer par des poids de récompense. La loi classique a
        besoin exactement du même amorçage, pour exactement la même raison : les
        deux lois valent zéro au repos.
        """
        cfg = self.cfg
        if getattr(cfg, "QL_KICK", 0.0) <= 0.0:
            return None
        if (abs(alpha_dot) < cfg.SWING_KICK_ADOT
                and abs(alpha) > math.pi - cfg.SWING_KICK_RAD):
            self.kick_time += cfg.RL_DT
            half = cfg.SWING_KICK_HALF_S
            forward = math.fmod(self.kick_time, 2.0 * half) < half
            return self.n_act - 1 if forward else 0
        self.kick_time = 0.0
        return None

    def potential(self, alpha: float, alpha_dot: float) -> float:
        """Potentiel énergétique Phi(s) pour le shaping POTENTIEL (Ng, Harada &
        Russell 1999).

        Le shaping n'est policy-invariant que sous la forme
        F(s,s') = gamma*Phi(s') - Phi(s). Un simple bonus d'état (ce que je
        faisais avant) n'a PAS cette propriété : l'agent l'accumule en RESTANT
        dans les états à haut potentiel, ce qui a créé un nouvel optimum local
        (faire tourner le pendule indéfiniment à E = eTop, 7500 de récompense
        par épisode sans jamais équilibrer). Sous forme de différence, le terme
        se télescope à zéro sur toute trajectoire fermée : tourner en rond ne
        rapporte plus rien, alors que le guidage le long d'un swing-up est
        identique.
        """
        cfg = self.cfg
        if cfg.QL_K_ENERGY <= 0.0:
            return 0.0
        e_top = max(cfg.eTop(), 1e-6)
        e = 0.5 * cfg.pendJ() * alpha_dot * alpha_dot + e_top * math.cos(alpha)
        return -cfg.QL_K_ENERGY * abs(e - e_top) / (2.0 * e_top)

    def approach_potential(self, alpha: float, alpha_dot: float,
                           theta_dot: float) -> float:
        """Potentiel lisse d'arrivée haute et lente, nul s'il est désactivé."""
        cfg = self.cfg
        k = float(getattr(cfg, "QL_K_APPROACH", 0.0))
        if k <= 0.0:
            return 0.0
        height = 0.5 * (1.0 + math.cos(alpha))
        wa = alpha_dot / max(float(cfg.QL_APPROACH_ADOT), 1e-6)
        wt = theta_dot / max(float(cfg.QL_APPROACH_TDOT), 1e-6)
        return k * height / (1.0 + wa * wa + wt * wt)

    # ---- Cycle de vie ----
    def start_session(self, greedy: bool) -> None:
        self.greedy = greedy
        self.prev_state_idx = -1
        self.steps_in_episode = 0.0
        self.episode_limit_s = float(self.cfg.QL_EPISODE_S)
        self.first_up_seen = False
        self.after_up_armed = False
        self.up_hold_time = 0.0
        self.episode_reward = 0.0
        self.paused = True
        self.pause_time = 0.0
        self.explore_hold = 0

    def _begin_pause(self) -> None:
        self.paused = True
        self.pause_time = 0.0
        self.explore_hold = 0

    def end_episode(self) -> None:
        self.last_episode_reward = self.episode_reward   # métrique simu uniquement
        if not self.greedy:
            self.episode += 1
            if self.episode_reward > self.best_reward:
                self.best_reward = self.episode_reward
            self.epsilon = max(self.cfg.QL_EPS_MIN, self.epsilon * self.cfg.QL_EPS_DECAY)
            self.learning_rate = max(
                float(getattr(self.cfg, "QL_LR_MIN", 0.0)),
                self.learning_rate * float(getattr(self.cfg, "QL_LR_DECAY", 1.0)))
        self.episode_reward = 0.0
        self.steps_in_episode = 0.0
        self.episode_limit_s = float(self.cfg.QL_EPISODE_S)
        self.first_up_seen = False
        self.after_up_armed = False
        self.up_hold_time = 0.0
        self.prev_state_idx = -1
        self.prev_phi = 0.0
        self.explore_hold = 0
        self.E.clear()

    def note_first_up(self, alpha: float) -> None:
        """Prolonge une seule fois l'épisode après sa première arrivée en haut."""
        near = abs(alpha) < float(getattr(self.cfg, "QL_FIRST_UP_RAD", 0.0))
        if not near:
            self.up_hold_time = 0.0
            return
        self.up_hold_time += self.cfg.RL_DT
        arm_s = float(getattr(self.cfg, "QL_AFTER_UP_ARM_S", 0.0))
        if self.up_hold_time >= arm_s:
            self.after_up_armed = True
        if not self.first_up_seen and self.steps_in_episode > 0.0:
            self.first_up_seen = True
            self.episode_limit_s += float(
                getattr(self.cfg, "QL_FIRST_UP_BONUS_S", 0.0))

    # ---- Un pas RL (20 Hz) ----
    def step(self, theta, alpha, theta_dot, alpha_dot, on_rezero) -> tuple[float, bool]:
        """Retourne (couple normalisé, fin d'épisode).

        `on_rezero` est appelé quand le firmware ferait Encoders::rezeroArm().
        """
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
            self.prev_state_idx = -1
            self.steps_in_episode = 0.0
            self.episode_reward = 0.0

        s_idx = self.state_index(alpha, alpha_dot, theta_dot)
        self.note_first_up(alpha)
        out_of_range, term_pen = self.terminal_check(theta, theta_dot, alpha)

        phi = (self.potential(alpha, alpha_dot)
               + self.approach_potential(alpha, alpha_dot, theta_dot))

        # SARSA(lambda) : il faut connaître l'action RÉELLEMENT choisie en s'
        # avant de mettre à jour la transition précédente, puisque le bootstrap
        # se fait sur Q(s', a') et non sur max_a Q(s', a).
        a_next = None
        if self.sarsa and not out_of_range:
            a_next = self._select_action(s_idx, alpha, alpha_dot)

        if not self.greedy and self.prev_state_idx >= 0:
            r = self.reward(alpha, alpha_dot, theta_dot, self.prev_action)
            # Shaping POTENTIEL : F = gamma*Phi(s') - Phi(s). Sur un état terminal
            # il n'y a pas de successeur, donc Phi(s') = 0 par convention.
            r += (0.0 if out_of_range else cfg.QL_GAMMA * phi) - self.prev_phi
            if out_of_range:
                r += term_pen
            self.last_step_reward = r
            self.episode_reward += r
            i = self.prev_state_idx + self.prev_action
            if out_of_range:
                target = r
            elif self.sarsa:
                # Valeur de la politique SUIVIE, exploration comprise.
                target = r + cfg.QL_GAMMA * self.Q[s_idx + a_next]
            else:
                target = r + cfg.QL_GAMMA * self.max_q(s_idx)
            delta = target - self.Q[i]
            if self.lam > 0.0:
                self._td_traces(i, delta, s_idx, out_of_range)
            else:
                self.Q[i] += self.learning_rate * delta

        if out_of_range:
            self.end_episode()
            self._begin_pause()
            self.u_command = 0.0
            return 0.0, True

        a = a_next if a_next is not None else self._select_action(s_idx, alpha, alpha_dot)

        # Watkins Q(lambda) : les traces sont COUPÉES après une action
        # exploratoire, sinon on créditerait des états pour un retour obtenu par
        # une action que la politique gloutonne n'aurait pas choisie.
        # /!\ SARSA(lambda) ne coupe PAS : il apprend justement la valeur de la
        # politique suivie, exploration incluse — couper les traces le ramènerait
        # à du hors-politique.
        if (self.lam > 0.0 and not self.greedy and not self.sarsa
                and a != self.best_action(s_idx)):
            self.E.clear()

        self.prev_state_idx = s_idx
        self.prev_action = a
        self.prev_phi = phi
        self.last_action = a - self.act_neutral
        self.u_command = self.action_u(a)

        self.steps_in_episode += 1.0
        if self.steps_in_episode * cfg.RL_DT >= self.episode_limit_s:
            self.end_episode()
            new_episode = True
            self._begin_pause()
        return self.action_u(a), new_episode

    # ---- Interopérabilité avec storage.cpp ----
    # struct QHeader { uint32_t magic; uint16_t version, nAlpha, nAdot, nAct,
    #                  reserved; };  -> 14 octets + 2 de padding = 16
    _HDR = "<IHHHHHxx"
    _MAGIC = 0x51544231  # 'QTB1'

    def save_bin(self, path: str | Path) -> None:
        """Écrit un /q_current.bin lisible tel quel par le firmware."""
        with open(path, "wb") as f:
            f.write(struct.pack(self._HDR, self._MAGIC, 1,
                                self.n_alpha, self.n_adot, self.n_act, 0))
            f.write(self.Q.tobytes())

    def load_bin(self, path: str | Path) -> None:
        with open(path, "rb") as f:
            hdr = f.read(struct.calcsize(self._HDR))
            magic, _ver, na, nd, nact, _res = struct.unpack(self._HDR, hdr)
            if magic != self._MAGIC:
                raise ValueError(f"{path} : magic invalide")
            if (na, nd, nact) != (self.n_alpha, self.n_adot, self.n_act):
                raise ValueError(
                    f"{path} : dimensions {na}x{nd}x{nact} != "
                    f"{self.n_alpha}x{self.n_adot}x{self.n_act} (config.h a changé)"
                )
            self.Q = array("f")
            self.Q.frombytes(f.read(4 * na * nd * nact))
