# Procédure de réglage complète — Pendule de Furuta

Réglage **from scratch** : mesure des paramètres physiques réels puis réglage
ordonné des gains pour un mode Classic (swing-up + équilibre) optimal.

Tout se règle **au menu → Reglages** (valeurs en RAM, `Sauver EEPROM` pour
persister). Quelques paramètres restent en dur dans `config.h` (signalés
« *compile-time* » ci-dessous) et nécessitent une recompilation.

> **Ordre impératif.** Chaque phase suppose la précédente validée. Ne saute pas,
> ne réordonne pas : le swing-up suppose l'équilibre réglé, qui suppose les
> signes bons, qui supposent les capteurs vérifiés.

---

## Phase 0 — Préparation & sécurité

- **Alim de debug** : Teensy alimenté par l'**USB du PC**, le **15 V uniquement
  sur le DRV8871** (masse commune). Évite le brownout/reboot pendant le réglage.
- **Condensateur** 1000–2200 µF sur l'entrée 15 V du driver (amortit les pics).
- **Arrêt** : un clic sur l'encodeur de menu coupe le moteur dans tout mode actif.
- Garde une **carte microSD** insérée : les sessions QL et les logs CSV
  (`/logs/`) aident à diagnostiquer, mais surtout tu pourras relire les courbes.
- Rappel : `DUTY_LIMIT` (menu `DutyLim`) est bas (0,30) pour la sécurité. On le
  remontera **à la fin**, progressivement.

---

## Phase 1 — Vérifier capteurs & signes (menu Debug)

Rien ne sert de régler des gains si les angles ou les sens sont faux.

1. **Debug → Angles live** (moteur coupé) :
   - Pendule immobile **en bas** → `a ≈ ±180°`. Pendule **en haut** → `a ≈ 0°`.
   - Sens : tourne le pendule dans le sens trigo (vu du dessus) → `a` doit
     **augmenter**. Sinon → `PendSign` (menu) à −1.
   - **Test anti-counts-ratés** : fais tourner bras et pendule **vite** à la
     main, plusieurs tours, reviens au départ → les counts (`cB`, `cP`) doivent
     revenir à ~0. S'ils dérivent → pull-ups externes 2,2–4,7 kΩ vers 3,3 V.
2. **Debug → Test mot. auto** (pendule retiré/bloqué) :
   - Attendu **`d+ > 0` et `d− < 0`**. Sinon → `ArmSign` (menu) à −1
     **ou** croiser les 2 fils moteur (l'un des deux, pas les deux).
3. **Menu → Recalibrer bas** : pendule **immobile en bas** ~2 s. À refaire
   après chaque reboot où le pendule n'était pas en bas au démarrage.
4. `Sauver EEPROM`.

---

## Phase 2 — Mesurer et saisir les paramètres physiques

Ces valeurs pilotent l'**énergie cible** du swing-up. Elles n'influencent **pas**
l'équilibre (retour d'état pur) — donc elles servent surtout à ce que le
pendule arrive **pile au sommet avec une vitesse faible**, prêt à être rattrapé.

> **Le « bras » n'est pas un paramètre du modèle.** Le firmware ne modélise que
> le **pendule** (masse, centre de masse, inertie). L'effet de la longueur du
> bras est absorbé empiriquement dans `Ke_swing` (Phase 5). Ne cherche pas à
> l'entrer quelque part.

Ce qu'il faut mesurer, sur la **partie qui oscille autour de l'axe du pendule**
(tige + masselotte, tout sauf le bras) :

### a) Masse `m` → `PendMass`
Pose l'ensemble pendule sur une **balance de cuisine**. Note en **kg**
(ex. 80 g → `0.080`).

### b) Centre de masse `L_com` → `PendLcom`
Distance **pivot du pendule → centre de masse**, en mètres.
Méthode : mets le pendule **en équilibre horizontal sur une arête** (crayon,
lame). Le point d'équilibre = centre de masse. Mesure la distance depuis le
pivot.

### c) Inertie → via `PendLen`
Le modèle calcule l'inertie autour du pivot par `J = m · PendLen² / 3`
(hypothèse tige homogène). Deux façons de renseigner `PendLen` :

**Option rapide (approximation tige)** : entre la **longueur physique** de la
tige dans `PendLen`, et `PendLcom = PendLen / 2` si la tige est bien homogène.
Suffisant pour démarrer.

**Option précise (méthode de la période) — recommandée** : elle capture
l'inertie réelle (tige + masselotte + fixations) en chronométrant le pendule.
1. Pendule suspendu, écarte-le d'un **petit angle** (~10–15°), lâche.
2. **Chronomètre `N` oscillations** complètes (N ≥ 20). Période `T = t / N` (s).
3. Calcule puis entre :

   ```
   J        = m · g · L_com · (T / 2π)²          (g = 9,81)
   PendLen  = √(3 · J / m) = (T / 2π) · √(3 · g · L_com)
   ```

   Exemple : `m=0.080`, `L_com=0.125`, `T=1.0 s`
   → `PendLen = (1.0/6.283)·√(3·9.81·0.125) ≈ 0.305`.

Puis `Sauver EEPROM`.

> Si le swing-up (Phase 5) **s'arrête juste sous la verticale**, ton `E_top` est
> un peu faible → augmente légèrement `PendMass` ou `PendLcom`. S'il **survole**
> le sommet trop vite → l'inverse.

---

## Phase 3 — Réglage moteur bas niveau

### Deadband — supprimé
Il n'y a **plus de compensation de frottement statique** : en FOC le couple est
lisse dès 0 tr/min, la zone morte du moteur brushed a disparu. Le paramètre a
été retiré du menu.

### `DutyLim` (`DUTY_LIMIT`) — plafond de couple/courant
Laisse à **0,30** pour tout le réglage. On le remontera en Phase 6.

### `DutySlew` (`DUTY_SLEW_PER_S`) — vitesse max de variation du duty
Limite la **rapidité** de changement du duty (en duty/s ; pas max par tick =
`DutySlew × 0,001`). **Compromis alim ↔ contrôle** :
- **Trop bas** = du retard dans la boucle. À `DutySlew = 3`, passer de +0,3 à
  −0,3 prend **200 ms** → équilibre mou (le moteur répond en retard, le pendule
  tombe malgré de bons gains), swing-up qui relance mal le bras.
- **Trop haut** = pics de courant → risque de **brownout** / coupure PD.

**C'est un garde-fou anti-brownout qui bride le contrôle.** Réglage :
- **Si le brownout est réglé côté matériel** (Teensy sur USB + condo) : monte-le
  franchement (**10–20 /s**) pour qu'il ne soit *jamais* la contrainte active
  pendant l'équilibre. C'est le cas recommandé.
- **Si tu dois protéger l'alim** (pas de découplage) : garde-le bas (3–5), en
  sachant que ça plafonne la performance — mieux vaut corriger l'alim.

Test : en Balance seul, si le pendule tombe alors que les gains semblent bons et
que le moteur « répond en retard » → augmente `DutySlew`. Si ça ramène le
brownout, tu as ton plafond → règle le problème d'alim plutôt que de brider ici.

### `Kp_vel` / `Ki_vel` — boucle de vitesse (sert au Q-learning, pas au Classic)
À ignorer pour le mode Classic. (Réglage : en QL greedy table vierge, action 0 =
bras immobile ; actions ±3 ≈ ±12 rad/s sans osciller.)

---

## Phase 4 — Réglage de l'équilibre (Balance seul)

But : tenir le pendule vertical, placé **à la main** près du haut (le contrôleur
s'arme sous ±34°). Loi : `u = −(K_alpha·α + K_adot·α̇ + K_th·θ + K_thd·θ̇)`.

Réglage un gain à la fois, dans cet ordre :

| Étape | Réglage | Critère |
|---|---|---|
| 1 | `K_th = 0`, `K_thd = 0` pour commencer (isole le pendule) | — |
| 2 | Monte **`K_alpha`** (6 → 8 → 10…) | jusqu'à ce qu'il **réagisse** et tienne quelques secondes ; s'il diverge violemment d'un coup, ton signe est faux (voir note ci-dessous) |
| 3 | Monte **`K_adot`** (0,3 → 0,6 → 0,9…) | jusqu'à **amortir les oscillations** ; trop haut = tremblement/bruit moteur |
| 4 | Ajoute un peu de **`K_thd`** (0,1–0,2) | amortit la rotation du bras, lisse la tenue |
| 5 | Ajoute un peu de **`K_th`** (0,05–0,15) | si le bras **dérive** lentement d'un côté ; ramène-le vers le centre. Le collecteur tournant autorise la rotation libre, donc `K_th = 0` est possible — mais une petite valeur évite que le bras parte en vadrouille pendant l'équilibre. |

Cible : le pendule tient **immobile** sans trembler ni dériver, et **rejette une
petite pichenette**. `Sauver EEPROM` dès que c'est bon.

> **Si dès `K_alpha` faible ça part violemment vers la chute** (le bras *fuit* au
> lieu de se placer dessous) : signe de contre-réaction inversé. Corrige avec
> `PendSign` puis **refais la Phase 1** (le sens de `a` change aussi).

*(Avancé : tu peux calculer un jeu de gains `[K_alpha K_adot K_th K_thd]` par LQR
à partir du modèle linéarisé, mais vu l'incertitude sur les paramètres, le
réglage manuel ci-dessus est plus rapide et plus fiable ici.)*

---

## Phase 5 — Réglage du swing-up (Classic)

Loi : `u = Ke_swing·(E − E_top)·α̇·cos(α) − Kthd_sw·θ̇`.

1. **Signe de `Ke_swing` d'abord.** S'il **amortit** le pendule vers le bas au
   lieu de le monter → passe `Ke_swing` en **négatif** (ex. +40 → **−40**).
   C'est LE réglage n°1 du swing-up.
2. **Magnitude ensuite** :

| Symptôme | Réglage |
|---|---|
| Pompe mais trop lentement / n'atteint pas le haut | augmente la magnitude de `Ke_swing` (−40 → −60 → −80…) |
| Pompe trop fort, dépasse violemment, faute `alpha_dot` (survitesse) | réduis la magnitude, ou baisse `AdotMax` seulement si tu veux couper plus tôt |
| Le **bras part en toupie** / dérive au lieu de rester sous le pendule | augmente `Kthd_sw` (0,004 → 0,008…) |
| Swing-up mou, bras trop freiné | réduis `Kthd_sw` |

3. **Objectif de raccord** : le pendule doit arriver **près de la verticale avec
   une vitesse `α̇` faible**, pour que la bascule vers l'équilibre s'accroche.
   La commutation se fait sous |α| < `BAL_ENTER_RAD` (~17°) **et**
   |α̇| < `BAL_ENTER_ADOT` (7 rad/s).
   - S'il **monte bien mais ne s'accroche pas** (passe trop vite au sommet) :
     ces deux seuils sont en dur (*compile-time*, `config.h`). Élargir
     `BAL_ENTER_ADOT` (7 → 9) aide à accrocher un passage rapide, au prix d'une
     bascule plus « brutale » à régler avec les gains d'équilibre.

`Sauver EEPROM` quand le cycle **swing-up → accroche → tient** fonctionne.

---

## Phase 6 — Optimisation & itération

1. **Remonte `DutyLim`** progressivement (0,30 → 0,35 → 0,40…) pour un rattrapage
   plus nerveux et un swing-up plus rapide. **Surveille le brownout** (reboot) et
   la température du driver à chaque palier ; recule au premier signe. Remonte
   aussi **`DutySlew`** (Phase 3) vers 10–20 si le brownout est réglé côté
   matériel : c'est souvent ce qui débride le plus la réactivité de l'équilibre.
2. **Relis les logs CSV** (`/logs/log_NNNN.csv`, colonnes
   `theta,alpha,theta_dot,alpha_dot,duty,…`) pour voir où ça décroche :
   overshoot au sommet, saturation du duty, oscillation résiduelle en équilibre.
3. **Affine en boucle** : un `DutyLim` plus haut peut demander de re-régler
   légèrement `K_adot`/`K_thd` (plus d'autorité = plus de tendance à osciller).
4. **Sauvegarde finale** : `Sauver EEPROM`. Note tes valeurs quelque part — un
   `Charger EEPROM` te les restaure, et `Defauts` + re-saisie te sert de filet.

---

## Récapitulatif des paramètres

| Menu Reglages | Rôle | Phase |
|---|---|---|
| `ArmSign`, `PendSign` | sens encodeurs/moteur | 1 |
| `PendMass`, `PendLcom`, `PendLen` | modèle énergie (swing-up) | 2 |
| `DutyLim`, `DutySlew` | moteur bas niveau | 3, 6 |
| `K_alpha`, `K_adot`, `K_th`, `K_thd` | équilibre | 4 |
| `Ke_swing`, `Kthd_sw` | swing-up | 5 |
| `Kp_vel`, `Ki_vel` | boucle vitesse (Q-learning) | — |
| `AdotMax`, `TdotMax`, `TurnsMax` | sécurités | 0, 5 |

**En dur dans `config.h` (recompilation) :** `BAL_ENTER_RAD`, `BAL_ENTER_ADOT`,
`BAL_EXIT_RAD` (seuils de bascule), `VEL_I_MAX`, `SAT_TIMEOUT_S`, dimensions et
hyperparamètres du Q-learning.
