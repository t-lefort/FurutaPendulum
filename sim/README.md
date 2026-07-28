# Simulation du pendule de Furuta

Bac à sable Python pour valider les réglages **avant** de les passer des heures sur
la machine. Un épisode de 15 s se simule en ~0,2 s : une campagne de 800 épisodes
(≈ 5 h de banc) tient en une minute, et 24 configurations tournent en parallèle.

La simulation **lit `config.h` directement** (`fw_config.parse_config_h`). Il n'y a
aucune copie des constantes : ce qui est validé ici est littéralement ce qui part
sur la Teensy, et modifier `config.h` change immédiatement la simu.

## Installation

Le cœur (physique, agent, entraînement, balayage) ne dépend **que de la
bibliothèque standard** :

```bash
python -m sim.train --check
```

`matplotlib` n'est nécessaire que pour la vue 3D et `--plot` :

```bash
pip install -r sim/requirements.txt
```

## Utilisation

### Audit statique — commencer par là

```bash
python -m sim.train --check
```

Ne simule rien : calcule le couple disponible, la vitesse de passage au point bas
exigée par un swing-up, la finesse des bins, l'horizon de `QL_GAMMA`, le jeu
d'actions, et liste les incohérences détectables (`consistency_report`).

### Entraînement

```bash
python -m sim.train --episodes 400 --plot runs/base.png
python -m sim.train --episodes 400 --save-q runs/q.bin
```

Avec `TC_SPLIT=1`, `--save-q` écrit le format `SPL1` lu directement par le
firmware : le fichier peut être copié sous `/q_current.bin` sur la microSD.
Le sidecar `/q_state.bin` est propre à un entraînement déjà commencé sur la
Teensy ; son absence est normale pour une politique issue de la simulation.

### Balayage de paramètres

```bash
python -m sim.train --sweep QL_U_MAX=0.55,0.70,0.85 --episodes 400 --jobs 8
python -m sim.train --sweep QL_LR=0.05,0.25 --sweep QL_GAMMA=0.99,0.995 --seeds 3 --jobs 12
```

Chaque axe est un `--sweep NOM=v1,v2,...`, le produit cartésien est exploré.
`--seeds N` moyenne N graines par configuration (indispensable : une seule graine
de RL ne prouve rien). Le tableau est trié par récompense moyenne des 50 derniers
épisodes.

### Référence classique

```bash
python -m sim.train --classic --set KE_SWING=50
```

Le swing-up énergétique répond à la question « ce montage peut-il physiquement
lever le pendule ? ». **Si le mode classique n'y arrive pas, aucun réglage RL n'y
arrivera** — l'agent ne peut pas dépasser la limite de couple. À lancer avant
toute campagne RL.

### Vue 3D

```bash
python -m sim.view3d --speed 5
python -m sim.view3d --mode classic --set KE_SWING=50
python -m sim.view3d --mode ql_greedy --load-q runs/q.bin --speed 3
```

`--speed N` = N secondes simulées par seconde d'horloge. Sert à **voir** ce que
les chiffres ne disent pas : le bras qui broute, le pendule amorti au lieu d'être
pompé, l'agent qui tourne en rond. Pour mesurer, utiliser `sim.train` (bien plus
rapide, sans rendu).

### Surcharges

`--set NOM=VAL` surcharge une constante de `config.h`, `--rig NOM=VAL` un
paramètre machine. Répétables, ou séparés par des virgules :

```bash
python -m sim.train --set QL_U_MAX=0.85,QL_U_MIN=0.20 --rig breakaway_duty=0.08
```

### Profil SARSA compact

`TC_SPLIT=1` active une fonction de valeur unique composée de deux banques de
traits sans hachage :

- une grille globale grossière pour le swing-up ;
- une grille locale plus fine autour de la verticale pour le rattrapage et
  l'équilibre.

Il s'agit toujours d'un seul agent SARSA, d'un seul jeu d'actions et d'une seule
mise à jour temporelle. Il n'y a ni contrôleur classique, ni bascule entre deux
politiques. Avec le profil robuste de `config.h`, les 16 656 traits et 7 actions
occupent 466 368 octets, soit 455,4 Kio de poids. Le tampon borné de 512 traces
ajoute 4 Kio en RAM1. La compilation Teensy 4.1 laisse 45 504 octets libres en
RAM2 et plus de 370 Kio disponibles pour les variables locales en RAM1.

Profil compact ayant appris depuis le bas, sans curriculum :

```bash
python -m sim.train --episodes 3000 --seed 1 --eval-every 500 \
  --set QL_SARSA=1,QL_K_ENERGY=8,QL_U_MIN=0.06,RL_DIVIDER=5,QL_GAMMA=0.999,QL_U_TAU=0.002,DUTY_SLEW_PER_S=80,QL_LAMBDA=0.92,QL_LR=0.01,QL_THETA_TURNS=6,QL_K_BAL=10,QL_BAL_CONE_TDOT=0,QL_EPS_DECAY=0.998,QL_EXPLORE_HOLD=24,QL_EXPLORE_NEAR_RAD=0.35,QL_EXPLORE_HOLD_TOP=1,QL_FIRST_UP_BONUS_S=0,TC_SPLIT_OVERLAP=0
```

Sur la graine 1, le probe glouton à 3 000 épisodes atteint 100 % de swing-up et
1,70 s de tenue médiane. Une réévaluation indépendante sur 30 épisodes donne
30/30 swing-ups, 25/30 tenues d'au moins une seconde, une médiane de 1,95 s et
un maximum de 7,6 s. Le fichier obtenu est
`runs/q_split_long_s1.bin` ; son ancien format compact n'a pas les dimensions
du profil robuste actuel et est donc correctement refusé par le firmware.

La convergence reste variable à pas constant. Les paramètres
`QL_LR_DECAY`/`QL_LR_MIN` et `QL_EXPLORE_EPS_TOP` servent aux campagnes de
robustesse : le premier évite qu'une bonne politique continue de dériver, le
second empêche les quatre actions aléatoires par seconde produites au sommet
par `epsilon=0.02` à 200 Hz.

`QL_FIRST_UP_BONUS_S` peut prolonger une seule fois l'épisode lors de sa
première arrivée près du sommet. Cette règle est directement reproductible sur
le banc : elle ne crée aucun état artificiel et laisse le même agent continuer
à apprendre sur sa propre trajectoire.

### Profil compact robuste (50 Hz)

Le profil validé reste un unique SARSA(lambda), sans contrôleur classique, sans
curriculum et avec chaque épisode démarrant pendule en bas. Les deux banques
sont seulement une factorisation de sa fonction Q. Le passage de 200 à 50 Hz
laisse à l'action demandée le temps d'être réellement appliquée.

```bash
python -m sim.train --episodes 3000 --seed 1 --eval-every 200 \
  --stop-when 20 --save-best-q runs/q_compact_best.bin \
  --save-q runs/q_compact_final.bin \
  --set QL_AFTER_UP_FALL_RAD=0.52,QL_AFTER_UP_ARM_S=1,QL_SARSA=1,QL_K_ENERGY=8,QL_K_APPROACH=20,QL_K_TDOT_TOP=0.1,QL_TDOT_TOP_RAD=0.35,QL_U_MIN=0.25,RL_DIVIDER=20,QL_GAMMA=0.995,QL_U_TAU=0.002,QL_DUTY_SLEW_PER_S=80,QL_LAMBDA=0.92,QL_LR=0.03,QL_LR_DECAY=0.9995,QL_LR_MIN=0.0005,QL_EPS0=0.30,QL_EPS_MIN=0.001,QL_EPS_DECAY=0.998,QL_EPS_TOP0=0.10,QL_EPS_TOP_MIN=0.001,QL_EPS_TOP_DECAY=0.9995,QL_THETA_TURNS=0,THETA_TURNS_MAX=0,QL_K_BAL=10,QL_BAL_CONE_TDOT=8,QL_EXPLORE_HOLD=6,QL_EXPLORE_NEAR_RAD=0.35,QL_EXPLORE_HOLD_TOP=1,QL_EXPLORE_EPS_TOP=0.30,QL_FIRST_UP_RAD=0.175,QL_FIRST_UP_BONUS_S=30,TC_SPLIT=1,TC_SPLIT_OVERLAP=0,TC_GLOBAL_TILINGS=8,TC_GLOBAL_N_ADOT=18,TC_GLOBAL_LR_SCALE=0.333333,TC_LOCAL_TILINGS=8,TC_LOCAL_TDOT_MAX=8,TC_LOCAL_LR_SCALE=2
```

La correction décisive est `QL_AFTER_UP_ARM_S=1`. Avant la première tenue
continue d'une seconde, une chute ne clôt pas l'épisode prolongé : l'agent
obtient plusieurs essais de rattrapage au lieu de refaire environ 11 secondes
de swing-up pour chaque transition locale. Dès qu'il sait tenir une seconde,
le terminal à 30 degrés est armé et la perte du bootstrap lui apprend à ne plus
retomber. Aucun état n'est fabriqué et aucune commande auxiliaire n'intervient ;
le mécanisme est directement portable sur la machine.

L'exploration est aussi séparée :

- epsilon global décroît par épisode pour apprendre le swing-up ;
- epsilon local part de 0,10 et décroît seulement lors d'une décision près du
  sommet ;
- le plancher vaut 0,001, car à 0,02 SARSA apprenait une politique qui comptait
  encore sur une correction aléatoire périodique.

Ralentir uniquement l'epsilon global (`0,9995`) a été mesuré comme moins bon :
les trois seeds conservaient 100 % de swing-up mais seulement 0,04 à 0,06 s
gloutonne vers 2 750 épisodes. Le problème était la densité des transitions de
rattrapage, pas un manque d'agitation pendant le pompage.

Trois entraînements indépendants depuis des poids nuls ont donné des meilleurs
probes gloutons de 43,30 s, 13,39 s et 27,71 s. Les checkpoints correspondants
sont `runs/q_dense_capture_s1.best.bin`,
`runs/q_dense_capture_s2.cont2.best.bin` et
`runs/q_dense_capture_s3.best.bin`. Sur 50 évaluations indépendantes par
checkpoint, sans exploration ni apprentissage :

| seed | swing-up | tenue >= 1 s | médiane | moyenne | minimum | maximum |
|---:|---:|---:|---:|---:|---:|---:|
| 1 | 50/50 | 50/50 | 42,60 s | 38,10 s | 9,80 s | 44,30 s |
| 2 | 50/50 | 50/50 | 13,40 s | 17,96 s | 1,30 s | 44,30 s |
| 3 | 50/50 | 50/50 | 31,25 s | 30,27 s | 1,70 s | 44,40 s |

La seed influence encore la vitesse de convergence et la qualité finale, mais
elle ne décide plus si l'équilibre est appris ou non : les trois répétitions
réussissent 50/50 swing-ups et 50/50 tenues d'au moins une seconde.

La configuration contient 16 656 traits : 10 944 globaux et 5 712 locaux.
Les poids occupent 466 368 octets (455,4 Kio). Avec 512 traces bornées
(environ 4 Kio), il reste environ 52 Kio sur les 512 Kio de RAM2 pour
l'en-tête et les autres allocations. Le rapport d'édition de liens du port
Teensy devra confirmer cette marge.

L'approximation linéaire peut régresser après un bon passage. L'option
`--save-best-q` conserve donc le meilleur probe glouton intermédiaire au lieu
de laisser la fin de campagne écraser une politique déjà validée.

## Les paramètres machine (`Rig`) — à mesurer

`config.h` ne contient pas la géométrie du bras ni les constantes moteur. Ils
vivent dans `Rig` (`fw_config.py`) avec des **ordres de grandeur plausibles, pas
des mesures**. Tant qu'ils ne sont pas recalés, la simu dit si un jeu de
paramètres RL est *cohérent*, pas s'il marchera au millimètre près.

Par ordre d'impact :

| paramètre | défaut | comment le mesurer |
|---|---|---|
| `breakaway_duty` | 0.15 | menu Debug > Jog manuel : monter de 5 % en 5 % jusqu'à ce que le bras parte. **Le plus important** : c'est lui qui décide quelles actions RL sont mortes. |
| `motor_kt` | 0.064 N.m/A | fiche du moteur, ou couple mesuré / courant mesuré. Fixe à la fois le couple à l'arrêt **et** la vitesse à vide (force contre-électromotrice) — les deux comptent. |
| `motor_R` | 10 Ω | ohmmètre entre deux phases, divisé par 2. Fixe le courant appelé, donc la marge sur l'alim 15 V / 3 A. |
| `arm_len` | 0.10 m | règle : axe vertical → pivot du pendule. |
| `arm_inertia` | 2.0e-4 kg.m² | autour de l'axe vertical, **rotor inclus** (inertie rotor × `MOTOR_GEAR_RATIO`²). |
| `pend_viscous`, `pend_coulomb` | faibles | temps de décroissance des oscillations libres : lâcher le pendule et compter les allers-retours. |

`pend_model` choisit la répartition de masse du pendule :

- `rod_bob` (défaut) — tige uniforme + masse au bout, dosées pour retomber sur
  `PEND_MASS`/`PEND_LCOM`/`PEND_LEN`. Le plus réaliste, et le seul des trois
  garanti physiquement cohérent.
- `point` — masse ponctuelle à `PEND_LCOM`.
- `firmware` — la formule de `Settings::pendJ()`. Avec
  `PEND_J_ROD_BOB=1`, elle coïncide maintenant avec `rod_bob`; avec `0`, elle
  revient au modèle historique de tige uniforme `m·L²/3`.

### Recaler la simu sur la machine

Ne pas croire les chiffres absolus tant que ça n'est pas fait :

1. Mesurer `breakaway_duty` au Jog.
2. Lancer le mode classique sur la machine, noter jusqu'où le pendule remonte.
3. Lancer `python -m sim.train --classic` avec le même `KE_SWING`, comparer.
4. Ajuster `motor_kt` / `arm_inertia` jusqu'à ce que les deux concordent.

À partir de là, les campagnes RL en simu prédisent utilement le banc.

### Approximation du pendule réel

Pour une tige imprimée de 10 cm estimée à 8 g et une vis de 8 g placée à son
extrémité, une approximation raisonnable est :

- masse totale : `0.016 kg` ;
- centre de masse : `0.075 m` sous le pivot ;
- inertie au pivot : `1.07e-4 kg.m²`.

La formule est `J = m_tige L²/3 + m_vis L²`. Le firmware SARSA utilise cette
approximation compilée pour rester identique au profil simulé. Le contrôle
classique continue, lui, d'utiliser ses paramètres runtime persistés en EEPROM :
son réglage validé au banc n'est donc pas écrasé par le portage RL.

Le même modèle peut être forcé explicitement en simulation :

```bash
python -m sim.train --check \
  --set PEND_MASS=0.016,PEND_LCOM=0.075,PEND_LEN=0.100,PEND_J_ROD_BOB=1
```

`PEND_J_ROD_BOB=1` utilise dans le firmware et la simulation
`J = m L (4 l_com - L) / 3`, la formule obtenue en déduisant une tige uniforme
et une masse en bout des trois grandeurs déjà présentes. Aucun paramètre
géométrique supplémentaire n'est à mesurer.

## Ce que la simu modélise

- **Équations de Lagrange complètes** du pendule de Furuta (couplage bras/pendule,
  termes centrifuges et de Coriolis), intégrées en RK4 à 1 kHz. `alpha = 0` est
  le haut, convention du firmware. La matrice de masse est vérifiée définie
  positive au démarrage : des paramètres mécaniquement impossibles font sortir en
  erreur plutôt que d'exploser numériquement 200 ticks plus tard.
- **Frottement stick–slip** du train d'engrenages : le bras reste *collé* tant que
  le couple demandé ne dépasse pas le décollement, puis passe en frottement
  cinétique (`kinetic_ratio`) plus visqueux. C'est ce qui rend les petites actions
  RL réellement inopérantes, comme sur la machine.
- **Moteur BLDC en mode couple** : `tau = gear·kt·(duty·MOTOR_VOLT_LIMIT −
  kt·gear·thetaDot) / R`. La force contre-électromotrice est incluse : au-delà
  d'une certaine vitesse de bras il ne reste plus de couple, ce qui borne le
  swing-up autant que le couple à l'arrêt.
- **Encodeurs** : quantification à 1000 PPR ×4, rapports de réduction, différence
  finie + filtre EMA — donc le même bruit de vitesse que le firmware (chiffré par
  `--check` en fraction d'un bin RL).
- **Chaîne de commande** : limitation de pente `DUTY_SLEW_PER_S`, écrêtage
  `DUTY_LIMIT`, lissage `QL_U_TAU`, cadencement RL à `RL_DIVIDER`, et l'ordre
  exact de `controlTick()` — la commande calculée à un tick n'agit qu'au suivant.
- **`Safety::check`** : mêmes seuils, mêmes codes de faute. Une faute est comptée
  puis le mode redémarre, pour qu'une campagne aille au bout ; un réglage qui
  faute sans arrêt reste un mauvais réglage.

## Ce qu'elle ne modélise pas

Jeu et élasticité des engrenages, commutation FOC (le couple est supposé suivre
la consigne instantanément à 10 kHz), déséquilibre du bras, flexion du pendule,
effondrement de l'alim en pointe de courant, échauffement.

## Correspondance avec le firmware

| module simu | module firmware |
|---|---|
| `fw_config.py` | `config.h`, `settings.h` (`Data::eTop`/`pendJ`) |
| `physics.py` | le montage réel + `encoders.cpp`, `motor.cpp` |
| `agent.py` | `qlearning.cpp` (port ligne à ligne) |
| `classic.py` | `control_classic.cpp` |
| `runner.py` | `controlTick()` (`FurutaPendulum.ino`) + `safety.cpp` |

`agent.py` reproduit les invariants de `qlearning.cpp` : égalité tranchée vers
`ACT_NEUTRAL`, pas de bootstrap sur un état terminal, contrôles terminaux sautés
au premier pas d'épisode, exploration maintenue `QL_EXPLORE_HOLD` pas. Toute
modification de `qlearning.cpp` doit être répercutée ici, sinon la simu valide
autre chose que ce qui tourne.
