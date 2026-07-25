# Pendule de Furuta — Teensy 4.1

Firmware pour pendule inversé rotatif : mode Classic (swing-up par énergie +
équilibre par retour d'état) et mode Q-learning tabulaire embarqué.
Moteur BLDC gimbal piloté en **FOC** (Field Oriented Control).

## Matériel

| Élément | Référence | Connexion |
|---|---|---|
| MCU | Teensy 4.1 | — |
| Driver moteur | **SimpleFOCMini** (DRV8313) | IN1=22, IN2=23, IN3=4, EN=5, VM=15 V, GND commun |
| Moteur | **BLDC gimbal** (type GBM2804, 12N14P, ~10 Ω) | phases A/B/C sur le driver |
| Encodeur bras | C38S6G5-1000B | A=0, B=1 (pull-ups 2,2–4,7k vers **3,3 V**) |
| Encodeur pendule | C38S6G5-1000B | A=2, B=3 (pull-ups 2,2–4,7k vers **3,3 V**) |
| Écran | GC9A01 240×240 rond | SCL=13, SDA=11, CS=10, DC=9, RST=8 |
| Encodeur menu | KY-040 | CLK=30, DT=31, SW=32, VCC=**3,3 V** |
| Collecteur tournant | sur l'axe vertical | passage des signaux de l'encodeur pendule → **rotation illimitée du bras** |
| Alimentation | USB PD 15 V / 3 A | SimpleFOCMini (VM) + buck 5 V → VIN Teensy |

**Pas de capteur dédié pour la FOC.** La commutation utilise l'**encodeur du
bras** : moteur et encodeur sont tous deux en 2:1 de l'axe vertical, donc ils
tournent au même rythme (1:1 entre eux). Aucun AS5600 n'est nécessaire.

> ⚠️ `MOTOR_POLE_PAIRS` (7 pour un 12N14P) doit correspondre au moteur, sinon
> l'alignement FOC échoue au démarrage (message « Align. KO »). Le moniteur
> série affiche une estimation (`PP check ... estimated pp:`) au boot.

## Réductions mécaniques

- Moteur → axe vertical : **2:1** (train d'engrenages externe)
- Encodeur bras → axe vertical : **2:1** → **4000 counts/tour moteur**,
  soit **8000 counts/tour d'axe** (0,045°)
- Encodeur pendule : prise directe, 4000 counts/tour

Tous les angles, vitesses, gains et limites du firmware sont exprimés **côté axe
vertical** (`ARM_ENC_RATIO` dans `config.h`).

## Menus

- **Classic** : Swing-up, Balance seul
- **Q-Learning** : Entrainer, Greedy, Sauver/Charger/Reset table
  (fonctionne **sans carte SD** — la Q-table vit en RAM, mais elle est alors
  perdue au reboot et aucun log n'est écrit)
- **Debug** : Angles live (angles + counts bruts, moteur coupé),
  Test moteur auto, Jog manuel, **Openloop** (fait tourner le moteur *sans
  capteur*, pour valider moteur/driver/câblage avant montage)
- **Reglages** : édition à l'écran de tous les paramètres de tuning,
  persistés en **EEPROM** (Sauver / Charger / Defauts)
- **Recalibrer bas** (pendule immobile en bas) — définit aussi θ = 0

Navigation : rotation = déplacer, clic = valider, appui long = retour au menu
principal. En mode actif, clic = arrêt moteur immédiat.

## Bibliothèques à installer (gestionnaire Arduino)

- **Simple FOC**
- **Adafruit GFX Library**
- **Adafruit GC9A01A**
- **Adafruit BusIO** (dépendance)
- `Encoder`, `SD` et `EEPROM` sont fournis avec Teensyduino.
- Optionnel : *Teensy-4.x-Quad-Encoder-Library* (mjs513, GitHub) pour le
  décodage quadrature matériel — décommenter `USE_HW_QUADENCODER` dans
  `encoders.h`.

Compilation : `arduino-cli compile --fqbn teensy:avr:teensy41 .`

## Mise en route

> Procédure complète (mesure des paramètres physiques réels + réglage ordonné
> de tous les gains) : **[REGLAGE.md](REGLAGE.md)**.

Au démarrage, le moteur **bouge tout seul 1–2 s** : c'est l'alignement FOC
(`initFOC()`). Laisser le bras libre et le **pendule immobile en bas**, car la
calibration (θ = 0 et α = π) est faite juste après.

Résumé :

1. **Debug → Angles live** : vérifier que `th` et `a` suivent, `a ≈ ±180°`
   pendule en bas et `≈ 0°` en haut. Sens faux → `PendSign` au menu Reglages.
2. **Debug → Test mot. auto** : attendu `d+ > 0` et `d− < 0`. Sinon `ArmSign`,
   **ou** permuter deux phases moteur (l'un des deux, pas les deux).
3. **Recalibrer bas**, pendule immobile.
4. **Balance seul** : placer le pendule à la main près de la verticale (le
   contrôleur ne s'arme que sous ±34°). Régler `K_alpha` puis `K_adot`.
5. **Classic** : si le pendule s'amortit au lieu de monter, **inverser le signe
   de `Ke_swing`**.
6. **QL → Entrainer** : épisodes de 15 s, logs CSV dans `/logs/`, Q-table
   auto-sauvée (`/q_current.bin`, `/q_best.bin`). Les 7 actions sont des
   **couples** appliqués directement au moteur (±`QL_U_MAX`), sans boucle de
   vitesse intermédiaire à régler.

## Sécurité

- **Clic sur l'encodeur menu = arrêt moteur immédiat** dans tous les modes.
- Coupures automatiques → écran **FAULT** affichant la cause :

  | Écran | Cause | Paramètre |
  |---|---|---|
  | `alpha_dot` | survitesse pendule | `AdotMax` |
  | `theta_dot` | survitesse bras | `TdotMax` |
  | `plage bras` | bras au-delà de ±N tours | `TurnsMax` (0 = illimité) |
  | `saturation` | commande saturée en continu trop longtemps | `SAT_TIMEOUT_S` |

  En FAULT le moteur est coupé et **le système y reste jusqu'à un clic**.
- **Un collecteur tournant équipe l'axe principal** : les signaux de l'encodeur
  pendule passent par la bague, rien ne s'enroule, et le bras peut donc tourner
  **indéfiniment** sans risque mécanique. `TurnsMax` peut être mis à **0
  (illimité)**. Le garder à une valeur élevée reste utile comme garde-fou
  anti-emballement (arrêt si le contrôle diverge et part en toupie).
- Fortement recommandé : interrupteur physique sur le 15 V.

## Réglages (menu « Reglages », persistés en EEPROM)

Les constantes de `config.h` ne sont plus que les **valeurs par défaut** (au
premier boot ou après « Defauts ») ; la valeur appliquée est `Settings::cfg`.

| Paramètre | Rôle |
|---|---|
| `ArmSign`, `PendSign` | sens des encodeurs / du moteur |
| `PendMass`, `PendLcom`, `PendLen` | modèle physique (énergie du swing-up) |
| `DutyLim`, `DutySlew` | couple max et vitesse de variation |
| `K_alpha`, `K_adot`, `K_th`, `K_thd` | équilibre (retour d'état) |
| `K_thi` | intégrale sur θ : débloque le bras coincé par le frottement |
| `Ke_swing`, `Kthd_sw` | swing-up par énergie |
| `AdotMax`, `TdotMax`, `TurnsMax` | seuils de sécurité |

`K_th` ramène le bras vers **θ = 0** (sa position au démarrage / à la dernière
recalibration). L'augmenter réduit la dérive du bras, mais trop fort il
déstabilise l'équilibre (système à non-minimum de phase) : compenser avec `K_thd`.

`K_thi` (défaut **0**) ajoute une **intégrale sur θ**. Utile quand le bras reste
planté loin de 0 : le frottement statique du train d'engrenages impose un seuil
de décollement, et une commande de quelques % ne suffit plus à le vaincre.
L'intégrale monte alors lentement jusqu'à débloquer le bras. Elle n'agit que
près de la verticale, et sa contribution est bornée à `TH_I_MAX` (anti-windup).
Dès que le bras est **revenu près de 0 et immobile** (`TH_I_DEAD_RAD` /
`TH_I_DEAD_DOT`), elle se **décharge** progressivement (`TH_I_FADE_S`) : plus
rien à débloquer, et ça évite le dépassement puis l'oscillation autour de 0.
Trop fort → le bras « chasse » (colle / décolle / dépasse) : réduire.

## Architecture

- `FurutaPendulum.ino` — machine à états ; ISR de contrôle 1 kHz ; **timer FOC
  dédié à 10 kHz** (priorité plus haute) ; UI/SD/logs dans `loop()`
- `config.h` — broches, ratios, constantes, valeurs par défaut
- `settings.*` — paramètres runtime + éditeur écran + persistance EEPROM
- `encoders.*` — angles + vitesses filtrées (θ, α, θ̇, α̇), α = 0 en haut ;
  calibration par **offset logiciel** (les compteurs bruts servent à la FOC)
- `motor.*` — BLDC en FOC (SimpleFOC) ; consigne = **couple normalisé** ;
  commutation dans son propre timer ; mode open-loop pour les tests au banc
- `safety.*` — survitesses, plage bras, saturation
- `control_classic.*` — swing-up énergie + retour d'état, commutation auto
- `qlearning.*` — Q-table 49×31×7 (float, DMAMEM), ε-greedy, épisodes ;
  sortie de plage du bras = fin d'épisode + retour automatique à θ ≈ 0
- `storage.*` — Q-table binaire + logs CSV sur microSD (optionnelle)
- `ui.*` — écran GC9A01 + encodeur KY-040 (menu, écrans live, fautes, réglages)
