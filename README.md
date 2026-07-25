# Pendule de Furuta — Teensy 4.1

Firmware pour pendule inversé rotatif : mode Classic (swing-up par énergie +
équilibre par retour d'état) et mode Q-learning tabulaire embarqué.

## Matériel

| Élément | Référence | Connexion |
|---|---|---|
| MCU | Teensy 4.1 | — |
| Driver moteur | DRV8871 (H-bridge) | IN1=22, IN2=23, VM=15V, GND commun, PWM 20 kHz |
| Moteur | RS-550VC 14,4 V | OUT1/OUT2 du DRV8871 |
| Encodeur bras | C38S6G5-1000B | A=0, B=1 (pull-ups 4,7k vers **3,3 V**) |
| Encodeur pendule | C38S6G5-1000B | A=2, B=3 (pull-ups 4,7k vers **3,3 V**) |
| Écran | GC9A01 240×240 | SCL=13, SDA=11, CS=10, DC=9, RST=8 |
| Encodeur menu | KY-040 | CLK=30, DT=31, SW=32, VCC=**3,3 V** |
| Alimentation | USB PD 15 V / 3 A | DRV8871 (VM) + buck 5 V → VIN Teensy |

## Réductions mécaniques

- Moteur → axe vertical : **2:1** (info seulement, le code commande un duty)
- Encodeur bras → axe vertical : **2:1** → résolution effective **8000 counts/tour d'axe** (0,045°)
- Encodeur pendule : prise directe, 4000 counts/tour

Tous les angles, vitesses, gains et limites du firmware sont exprimés **côté axe vertical** (`ARM_ENC_RATIO` dans `config.h`).

## Menus

- **Classic** : Swing-up, Balance seul
- **Q-Learning** : Entrainer, Greedy, Sauver/Charger/Reset table
- **Debug** : Angles live (angles + counts bruts, moteur coupé),
  Test moteur auto (2 s à +12 %, pause, 2 s à −12 %, affiche Δθ par sens —
  attendu : Δ+ > 0 et Δ− < 0, sinon inverser `ARM_SIGN`), Jog manuel
- **Recalibrer bas** (pendule immobile en bas)

Navigation : rotation = déplacer, clic = valider, appui long = retour au menu principal. En mode actif, clic = arrêt moteur immédiat.

## Bibliothèques à installer (gestionnaire Arduino)

- **Adafruit GFX Library**
- **Adafruit GC9A01A**
- **Adafruit BusIO** (dépendance)
- `Encoder` et `SD` sont fournis avec Teensyduino.
- Optionnel : *Teensy-4.x-Quad-Encoder-Library* (mjs513, GitHub) pour le
  décodage quadrature matériel — décommenter `USE_HW_QUADENCODER` dans
  `encoders.h`. La version par défaut (interruptions) suffit largement pour
  valider tout le montage.

## Mise en route (ordre impératif)

> Pour un **réglage complet from-scratch** (mesure des paramètres physiques réels
> + réglage ordonné de tous les gains), voir **[REGLAGE.md](REGLAGE.md)**. Le
> résumé ci-dessous en est la version courte.

1. **Sans le moteur alimenté** (15 V débranché) : téléverser, ouvrir le
   moniteur série. Bouger le bras et le pendule à la main : `th` et `a`
   doivent suivre. Pendule en bas immobile → `a ≈ ±3.14`. Pendule en haut →
   `a ≈ 0`. Si `a` diminue quand le pendule tourne dans le sens trigo (vu du
   dessus), inverser `PEND_SIGN` dans `config.h`.
2. **Menu → Test moteur** (15 V branché, pendule retiré ou bloqué au début) :
   la rotation de l'encodeur menu ajuste le duty par pas de 5 %. Une commande
   positive doit faire **augmenter** `th`. Sinon, inverser `ARM_SIGN` **ou**
   croiser les fils moteur (pas les deux).
3. **Recalibrer (bas)** dans le menu : pendule immobile, pendant ~2 s.
   À faire après chaque reset si le pendule n'était pas en bas au boot.
4. **Balance seul** : placer le pendule à la main près de la verticale ;
   le contrôleur ne s'active que sous ±34°. Régler `K_ALPHA`/`K_ADOT`
   jusqu'à tenue stable (augmenter `K_ALPHA` si mou, `K_ADOT` si oscillant).
5. **Classic swing-up** : si le pendule s'amortit au lieu de monter,
   inverser le signe de `KE_SWING`.
6. **QL: entrainer** : lancer des épisodes de 15 s. Logs CSV dans `/logs/`,
   Q-table auto-sauvée (`/q_current.bin`, `/q_best.bin`).

## Sécurité

- **Clic sur l'encodeur menu = arrêt moteur immédiat** dans tous les modes.
- Coupures automatiques : survitesse pendule/bras, bras au-delà de
  `THETA_TURNS_MAX` tours (mettre 0 si collecteur tournant), duty saturé > 3 s.
- Fortement recommandé : interrupteur physique sur le 15 V entre la carte
  USB PD et le DRV8871.
- `DUTY_LIMIT = 0.30` protège la source USB PD (3 A). À augmenter
  progressivement en surveillant que la carte PD ne coupe pas.

## Réglages principaux (`config.h`)

- `PEND_MASS`, `PEND_LCOM`, `PEND_LEN` : mesurer sur le pendule réel
  (le swing-up par énergie en dépend directement).
- Deadband : supprimé (inutile en FOC, le couple est lisse dès 0 tr/min).
- Gains balance/swing-up : voir procédure ci-dessus.
- `KP_VEL`/`KI_VEL` : boucle de vitesse pour le Q-learning. Test : en mode
  QL greedy avec table vierge, action 0 → le bras doit rester immobile ;
  les actions ±3 doivent donner ~±12 rad/s sans oscillation.

## Architecture

- `FurutaPendulum.ino` — machine à états, boucle 1 kHz (IntervalTimer), UI/SD dans `loop()`
- `config.h` — broches, constantes, gains
- `encoders.*` — angles + vitesses filtrées (θ, α, θ̇, α̇), α=0 en haut
- `motor.*` — PWM 20 kHz (DRV8871), deadband, slew rate, limite duty, PI vitesse
- `safety.*` — survitesses, plage bras, saturation
- `control_classic.*` — swing-up énergie + retour d'état, commutation auto
- `qlearning.*` — Q-table 49×31×7 (float, DMAMEM), ε-greedy, épisodes
- `storage.*` — Q-table binaire + logs CSV sur microSD
- `ui.*` — écran GC9A01 + encodeur KY-040 (menu, écrans live, fautes)
