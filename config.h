#pragma once
#include <Arduino.h>

// ============================================================
//  Pendule de Furuta — Teensy 4.1
//  Toute la configuration matérielle et les gains sont ici.
//
//  NOTE : les constantes de réglage (signes, gains, limites moteur, modèle
//  physique, limites de sécurité) servent désormais de VALEURS PAR DÉFAUT.
//  La valeur réellement appliquée est Settings::cfg.<champ> (settings.*),
//  éditable au menu "Reglages" et persistée en EEPROM. Modifier ici = changer
//  le défaut compilé (utilisé au premier boot / après "Defauts").
// ============================================================

// ---------- Broches ----------
// Encodeurs incrémentaux (décodeur quadrature MATÉRIEL, broches XBAR)
constexpr uint8_t PIN_ENC_ARM_A  = 0;   // encodeur bras (moteur)
constexpr uint8_t PIN_ENC_ARM_B  = 1;
constexpr uint8_t PIN_ENC_PEND_A = 2;   // encodeur pendule
constexpr uint8_t PIN_ENC_PEND_B = 3;

// Driver BLDC SimpleFOCMini (DRV8313) : 3 sorties PWM + enable
constexpr uint8_t PIN_DRV_PWM_A = 22;
constexpr uint8_t PIN_DRV_PWM_B = 23;
constexpr uint8_t PIN_DRV_PWM_C = 4;
constexpr uint8_t PIN_DRV_EN    = 5;
// Capteur de position moteur pour la FOC : on REUTILISE l'encodeur du bras.
// Moteur et encodeur bras sont tous deux en 2:1 de l'axe vertical -> ils
// tournent au meme rythme (1:1 entre eux), donc l'encodeur bras donne
// directement l'angle de l'arbre moteur. Pas de capteur supplementaire.

// Écran GC9A01 (SPI0 matériel : SCK=13, MOSI=11)
constexpr uint8_t PIN_TFT_CS  = 10;
constexpr uint8_t PIN_TFT_DC  = 9;
constexpr uint8_t PIN_TFT_RST = 8;

// Encodeur rotatif de menu (KY-040 : CLK/DT/SW)
constexpr uint8_t PIN_UI_CLK = 30;
constexpr uint8_t PIN_UI_DT  = 31;
constexpr uint8_t PIN_UI_SW  = 32;

// ---------- Capteurs ----------
constexpr float ENC_CPR  = 4000.0f;              // 1000 PPR x4 (quadrature)
constexpr float CNT2RAD  = TWO_PI / ENC_CPR;

// Rapports de réduction (tours du composant PAR TOUR de l'axe vertical)
constexpr float ARM_ENC_RATIO    = 2.0f;   // encodeur bras : 2 tours / tour d'axe -> 8000 cts/tour d'axe
constexpr float MOTOR_GEAR_RATIO = 2.0f;   // moteur : 2 tours / tour d'axe (info, non utilisé par le code)
constexpr float ARM_CNT2RAD  = CNT2RAD / ARM_ENC_RATIO;
constexpr float PEND_CNT2RAD = CNT2RAD;    // pendule en prise directe sur son encodeur

// Angle de l'ARBRE MOTEUR vu par l'encodeur bras (pour la commutation FOC).
// Tours d'encodeur par tour de moteur = ARM_ENC_RATIO / MOTOR_GEAR_RATIO = 1.0.
// -> 4000 cts/tour moteur, soit ~571 cts par tour electrique a 7 paires de poles.
constexpr float MOTOR_ENC_RATIO = ARM_ENC_RATIO / MOTOR_GEAR_RATIO;
constexpr float MOTOR_CNT2RAD   = CNT2RAD / MOTOR_ENC_RATIO;

// Signes à ajuster pendant la calibration (menu "Test moteur") : +1.0f ou -1.0f
// Convention : theta > 0 = bras dans le sens de la commande positive
//              alpha = 0 pendule EN HAUT, alpha > 0 dans le sens trigo vu du dessus
constexpr float ARM_SIGN  = +1.0f;
// VALIDE AU BANC : avec PEND_SIGN = +1 et KE_SWING = -50, le swing-up et
// l'equilibre du mode classique fonctionnent tres bien sur la machine reelle.
// Ne pas "corriger" ce signe sur la foi de la simulation seule : c'est la
// convention geometrique de sim/physics.py qui doit s'aligner sur la machine
// (cf. Rig.coupling_sign), pas l'inverse.
constexpr float PEND_SIGN = +1.0f;

// ---------- Moteur BLDC / FOC ----------
// Moteur gimbal GBM2804 : 12N14P -> 14 aimants -> 7 paires de poles.
// A CORRIGER si tu changes de moteur (une valeur fausse empeche l'alignement FOC).
constexpr int   MOTOR_POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE   = 15.0f;   // alim du SimpleFOCMini
// Tension q max appliquee au moteur. Sur un GBM2804 (~10 ohm) : 6 V -> 0,6 A,
// tres large sous les 3 A du driver.
// VALIDE AU BANC : 6 V suffisent, le swing-up classique passe. Noter que la
// commande appliquee vaut duty * MOTOR_VOLT_LIMIT : c'est ce produit qui fixe le
// couple, DUTY_LIMIT seul ne veut rien dire.
constexpr float MOTOR_VOLT_LIMIT = 6.0f;
constexpr float FOC_PWM_FREQ_HZ  = 25000.0f; // inaudible
// Frequence de la commutation FOC (timer dedie). Doit etre BIEN plus rapide que
// la boucle de controle : c'est elle qui fabrique les tensions de phase.
// Trop lente -> commutation en escalier = moteur saccade.
constexpr float FOC_FREQ_HZ      = 10000.0f;

// Limites de commande. La "duty" est desormais un COUPLE NORMALISE [-1, 1]
// qui est mis a l'echelle par MOTOR_VOLT_LIMIT.
constexpr float DUTY_LIMIT      = 0.90f;  // fraction max du couple dispo
// Variation max par seconde du controle classique. Le RL utilise sa propre limite
// QL_DUTY_SLEW_PER_S afin de ne pas modifier le comportement valide au banc.
constexpr float DUTY_SLEW_PER_S = 40.0f;
// (La compensation de zone morte a ete supprimee : la FOC est lisse des
//  0 tr/min. Le frottement statique de la mecanique est traite par le terme
//  integral K_TH_I plus bas.)

// ---------- Boucles ----------
constexpr float    CTRL_FREQ_HZ = 1000.0f;          // boucle de contrôle
constexpr float    CTRL_DT      = 1.0f / CTRL_FREQ_HZ;
// SARSA a 1000/20 = 50 Hz. Cette cadence echantillonne correctement le passage
// rapide au sommet ; le lissage et le slew rate propres au RL protegent le
// train d'engrenages contre les inversions d'actions discretes.
constexpr uint32_t RL_DIVIDER   = 20;
constexpr float    RL_DT        = CTRL_DT * RL_DIVIDER;
constexpr float    VEL_FILT_ALPHA = 0.20f;          // passe-bas vitesses (0..1, plus grand = moins filtré)

// ---------- Sécurité ----------
// Un swing-up EXIGE de passer au point bas a sqrt(4*m*g*lcom/J) rad/s, soit
// ~27 rad/s avec les valeurs par defaut du modele. Couper a 30 ne laissait que
// 3 rad/s de marge : un swing-up un peu trop energique tombait en faute.
// Valeurs utilisees au banc (Reglages) : AdotMax 40, TdotMax 40.
// /!\ THETA_DOT_MAX doit rester STRICTEMENT au-dessus de QL_TDOT_MAX, sinon la
// securite coupe le mode avant le terminal d'episode : l'agent tombe en faute au
// lieu d'apprendre de la penalite terminale.
constexpr float ALPHA_DOT_MAX   = 40.0f;  // rad/s — coupure si dépassé
constexpr float THETA_DOT_MAX   = 40.0f;  // rad/s
// Un COLLECTEUR TOURNANT equipe l'axe : les fils de l'encodeur pendule ne
// s'enroulent pas, le bras peut tourner indefiniment. Cette limite n'est donc
// PAS une protection du cablage, juste un garde-fou anti-emballement.
// 0 = illimite.
constexpr float THETA_TURNS_MAX = 30.0f;   // tours max du bras (valeur banc)
constexpr float SAT_TIMEOUT_S   = 8.0f;   // coupure si duty saturé en continu trop longtemps

// ---------- Modèle physique (pour le swing-up énergie) ----------
// À ajuster avec tes valeurs réelles (masse pendule, distance pivot -> centre de masse)
// Approximation du montage actuel : bras imprime d'environ 8 g sur 10 cm
// + vis d'environ 8 g au bout. Cela donne m_total ~= 16 g et
// l_com = (8 g * 5 cm + 8 g * 10 cm) / 16 g = 7,5 cm.
constexpr float PEND_MASS  = 0.016f;                 // kg
constexpr float PEND_LCOM  = 0.075f;                 // m (pivot -> centre de masse)
constexpr float PEND_LEN   = 0.100f;                 // m (longueur totale)
// 0 conserve exactement le calcul historique m*L²/3 et donc le réglage
// classique validé au banc. 1 suppose une tige uniforme + une masse ponctuelle
// au bout, dont les masses sont déduites de (masse totale, LCOM, longueur) :
// J = m*L*(4*LCOM-L)/3. Pour ~8 g de tige sur 10 cm + vis de 8 g au bout,
// utiliser M=.016, LCOM=.075, L=.100 donne J=1.067e-4 kg.m².
constexpr int   PEND_J_ROD_BOB = 1;
constexpr float G_GRAV     = 9.81f;
// Énergie cible E_TOP et inertie PEND_J sont dérivées de la masse/longueur :
// elles vivent maintenant dans Settings (cfg.eTop() / cfg.pendJ()) car ces
// paramètres sont réglables au menu.

// ---------- Gains mode Classic (POINTS DE DÉPART, à régler) ----------
// Swing-up : u = KE_SWING * (E - E_TOP) * alpha_dot * cos(alpha) - KTHD_SWING * theta_dot
constexpr float KE_SWING   = -50.0f;
constexpr float KTHD_SWING = 0.004f;
// Amorcage : au repos EXACT en bas, alpha_dot = 0 donc la loi d'energie rend
// u = 0 et rien ne demarre jamais (point stationnaire de la commande). On
// applique une petite impulsion ALTERNEE pour briser la symetrie ; elle pompe
// le pendule jusqu'a ce qu'il bouge, puis la loi d'energie reprend la main.
constexpr float SWING_KICK_U      = 0.18f;  // couple d'amorcage
constexpr float SWING_KICK_ADOT   = 0.30f;  // rad/s en dessous = "immobile"
constexpr float SWING_KICK_RAD    = 0.25f;  // rad autour de +/-pi = "en bas"
constexpr float SWING_KICK_HALF_S = 0.25f;  // demi-periode d'alternance

// Équilibre (retour d'état, sortie = duty) :
// u = -(K_ALPHA*alpha + K_ADOT*alpha_dot + K_TH*theta + K_THD*theta_dot)
constexpr float K_ALPHA = 9.0f;
constexpr float K_ADOT  = 0.60f;
constexpr float K_TH    = 0.20f;   // 0 = ignorer theta (rotation libre, cf. collecteur)
constexpr float K_THD   = 0.42f;
// Terme INTEGRAL sur theta. Sert a vaincre le frottement statique du train
// d'engrenages : quand la commande proportionnelle tombe sous le seuil de
// decollement, le bras reste coince loin de 0 et l'integrale monte jusqu'a le
// debloquer. Defaut 0 = desactive : a augmenter progressivement au menu.
constexpr float K_TH_I  = 0.055f;
// Contribution max du terme integral a la commande (anti-windup).
constexpr float TH_I_MAX = 0.25f;
// Zone morte du terme integral : quand le bras est revenu pres de 0 ET qu'il
// est a l'arret, il n'y a plus de frottement a vaincre. L'integrale se
// DECHARGE alors (au lieu de continuer a pousser -> depassement / oscillation).
// Decharge progressive et non brutale : annuler d'un coup ferait un saut de
// commande pouvant atteindre TH_I_MAX, qui secouerait le pendule.
constexpr float TH_I_DEAD_RAD = 0.10f;  // rad (~6 deg) : "bras a la maison"
constexpr float TH_I_DEAD_DOT = 0.30f;  // rad/s        : "bras immobile"
constexpr float TH_I_FADE_S   = 0.30f;  // s : constante de temps de decharge

// Commutation swing-up <-> équilibre
constexpr float BAL_ENTER_RAD  = 0.30f;  // ~17°
constexpr float BAL_ENTER_ADOT = 7.0f;   // rad/s
constexpr float BAL_EXIT_RAD   = 0.60f;  // ~34°

// ---------- Q-learning ----------
constexpr int   QL_N_ALPHA  = 49;
constexpr int   QL_N_ADOT   = 41;
constexpr int   QL_N_ACT    = 7;
// 3e dimension d'etat OPTIONNELLE : vitesse du BRAS. A 1, l'etat reste
// [alpha, alpha_dot] (comportement historique). Sans theta_dot dans l'etat,
// l'agent ne peut PAS representer un rattrapage : la loi d'equilibre classique
// a besoin du terme K_THD*theta_dot (verifie en simu : a K_THD = 0 l'equilibre
// tombe). Le prix est multiplicatif sur la taille de table, qui vit en DMAMEM :
// N_ALPHA*N_ADOT*N_TDOT*N_ACT*4 octets doit rester sous ~400 kB.
constexpr int   QL_N_TDOT       = 1;
constexpr float QL_TDOT_BIN_MAX = 12.0f;  // rad/s, plage de discretisation
// Plage de discretisation de alpha_dot. DOIT couvrir la vitesse de passage au
// point bas exigee par un swing-up (~27 rad/s avec le modele par defaut) :
// a 20 rad/s les bins saturaient exactement dans le regime qui compte, l'agent
// etait aveugle au moment decisif (impossible de distinguer "20 rad/s" de
// "26 rad/s, ca va passer").
constexpr float QL_ADOT_MAX = 28.0f;   // rad/s, plage de discrétisation
// Les actions sont des COUPLES NORMALISES appliques directement au moteur (pas
// de boucle de vitesse intermediaire). Une consigne de vitesse rendait le
// processus non markovien : le couple reellement applique dependait de l'etat
// du PI et de theta_dot, dont aucun n'est observe par l'agent.
//
// Les niveaux non nuls sont repartis sur [QL_U_MIN, QL_U_MAX] et NON sur
// [0, QL_U_MAX] : sous le seuil de decollement du train d'engrenages une action
// ne produit AUCUN mouvement. Reparties lineairement depuis 0, les deux petites
// actions etaient des "ne rien faire" plus cher que l'action neutre -> 3 actions
// sur 7 physiquement identiques, et 29 % de l'exploration gaspillee.
// QL_U_MIN se MESURE : menu Debug > Jog manuel, monter la duty de 5 % en 5 %
// jusqu'a ce que le bras decolle, puis prendre un cran au-dessus.
constexpr float QL_U_MIN    = 0.25f;   // couple minimal qui met reellement en mouvement
constexpr float QL_U_MAX    = 0.70f;   // couple normalise max commande par le RL
// Limiteur propre au RL : ne modifie pas le DutySlew du controle classique.
constexpr float QL_DUTY_SLEW_PER_S = 80.0f;
// Lissage du couple RL (1er ordre). Les actions sont discretes et peuvent
// s'inverser d'un pas a l'autre : applique brut, ca fait claquer la mecanique.
// Une constante de temps courte devant la periode d'action (20 ms a 50 Hz)
// arrondit les fronts sans introduire de retard notable.
constexpr float QL_U_TAU    = 0.002f;  // s
constexpr float QL_LR       = 0.03f;   // learning rate
// Décroissance par épisode. À pas constant, la petite représentation compacte
// continue de déplacer une politique déjà bonne : une tenue gloutonne de 0,62 s
// à l'épisode 2000 retombait à 0,00 s à 2500. Le plancher permet de continuer à
// s'adapter lentement sur le vrai montage.
constexpr float QL_LR_DECAY = 0.9995f;
constexpr float QL_LR_MIN   = 0.0005f;
// Horizon : gamma^n a 50 Hz. 0.97 -> demi-vie ~0,45 s : l'agent est bien trop
// myope pour "voir" un swing-up qui dure plusieurs secondes, il prend donc la
// recompense immediate. 0.99 -> ~1,4 s ; 0.995 -> ~2,8 s.
constexpr float QL_GAMMA    = 0.995f;
constexpr float QL_EPS0     = 0.30f;
constexpr float QL_EPS_MIN  = 0.001f;
constexpr float QL_EPS_DECAY = 0.998f; // par épisode
// Exploration locale indépendante. Contrairement à QL_EPS_DECAY, cette valeur
// ne décroît que lorsqu'une décision est réellement prise près du sommet. Une
// seed qui atteint rarement le haut conserve ainsi son budget d'exploration.
// QL_EPS_TOP0 < 0 garde le comportement historique QL_EXPLORE_EPS_TOP.
constexpr float QL_EPS_TOP0      = 0.10f;
constexpr float QL_EPS_TOP_MIN   = 0.001f;
constexpr float QL_EPS_TOP_DECAY = 0.9995f; // par décision locale
constexpr float QL_EPISODE_S = 15.0f;  // durée d'un épisode
// Une arrivee au sommet est rare et survient souvent tard dans l'episode.
// Ajouter UNE fois cette fenetre a la premiere entree pres du haut evite de
// couper l'apprentissage au moment precis ou les echantillons d'equilibre
// deviennent disponibles. Portable sur la machine : aucun reset artificiel,
// aucun controleur auxiliaire, l'agent continue simplement son episode.
constexpr float QL_FIRST_UP_RAD     = 0.175f;
constexpr float QL_FIRST_UP_BONUS_S = 30.0f;  // episode total <= 45 s
// Apres le premier rattrapage, une chute peut clore immediatement la tentative.
// Chaque episode repart toujours du bas : ce n'est ni un curriculum, ni un
// controleur auxiliaire. 0 = desactive ; 0.52 rad ~= 30 deg.
constexpr float QL_AFTER_UP_FALL_RAD = 0.52f;
// N'armer cette terminaison qu'apres une premiere tenue continue. Avant cela,
// l'episode prolonge peut fournir plusieurs tentatives de rattrapage.
// 0 = armee des la premiere entree dans QL_FIRST_UP_RAD.
constexpr float QL_AFTER_UP_ARM_S = 1.0f;
// Exploration PERSISTANTE : une action aleatoire est TENUE pendant N pas RL.
// Tiree a nouveau a chaque pas, l'exploration epsilon-greedy est un bruit blanc
// a 50 Hz : sa moyenne est nulle, elle ne peut donc pas POMPER un oscillateur a
// ~1,5 Hz (il faut ~1/3 de s de couple dans le meme sens). L'agent ne rencontre
// jamais un debut de swing-up, donc ne peut pas l'apprendre.
// 6 pas = 120 ms ; pres du sommet QL_EXPLORE_HOLD_TOP reprend la main.
constexpr int   QL_EXPLORE_HOLD = 6;

// --- Fin d'episode sur sortie de plage du bras ---
// Sans rapport avec le collecteur tournant (la rotation est libre) : c'est une
// limite d'EPISODE. Elle donne un etat de depart coherent a chaque episode et
// une penalite terminale qui decourage la strategie "tourner a fond".
// A distinguer de TurnsMax, qui coupe tout le mode : ici l'episode se termine
// et un nouveau redemarre automatiquement, l'entrainement continue.
// Mesures RELATIVEMENT a la position du bras au debut de l'episode : ce qui
// compte est de ne pas laisser un episode partir en vrille, pas la position
// absolue (theta n'est pas observe, et le collecteur autorise tout).
constexpr float QL_THETA_TURNS = 0.0f;    // collecteur tournant : illimite
// Termine aussi l'episode si le bras s'emballe. Le moteur etant coupe ensuite,
// le bras finit sur son elan, donc un seuil bas limite la roue libre.
// /!\ NE PAS REDESCENDRE CE SEUIL. Il a valu 8 puis 14 rad/s, et les deux
// etaient SOUS la vitesse de bras qu'exige un swing-up reel : en simulation, a
// 14 rad/s, 622 episodes sur 800 (78 %) se terminaient sur ce terminal et le
// taux de swing-up tombait a 1 %. Le bras passe legitimement a ~29 rad/s de
// pointe pendant un pompage. Mesure (2500 episodes, tout le reste egal) :
//   TDOT_MAX 14 + K_TDOT 1,5 -> recompense -11,5, swing-up  1 %
//   TDOT_MAX 25 + K_TDOT 0   -> recompense  82,6, swing-up  7 %
//   TDOT_MAX 40 + K_TDOT 0   -> recompense  93,4, swing-up 27 %
// La securite globale (ThetaDotMax) doit rester au-dessus.
constexpr float QL_TDOT_MAX    = 30.0f;   // rad/s (0 = desactive), < THETA_DOT_MAX
// Barriere DOUCE avant la limite dure : au-dela de QL_TDOT_SOFT la recompense
// decroit lineairement (pente QL_K_TDOT). theta_dot n'etant PAS dans l'etat, un
// malus est de toute facon peu attribuable par l'agent ; restreint aux tres
// grandes vitesses il reste correle aux actions recentes, mais la mesure
// ci-dessus est sans appel : a pente non nulle le swing-up ne s'apprend pas
// (0 a 2 % contre 27 %). Le mecanisme est conserve, la pente est mise a zero.
// Ne la remonter qu'avec une campagne sim/ a l'appui.
constexpr float QL_TDOT_SOFT   = 25.0f;   // rad/s : debut de la penalite
constexpr float QL_K_TDOT      = 0.0f;    // par (rad/s) au-dela du seuil
constexpr float QL_R_OUT_RANGE = -50.0f;  // penalite terminale

// --- Bonus "pendule en haut" (cf. reward() dans qlearning.cpp) ---
// Les gates sur |theta'| empechent l'exploit centrifuge (bras qui tourne a fond
// et tient le pendule releve sans jamais equilibrer). Ils doivent rester
// ATTEIGNABLES : a l'arrivee en haut le bras tourne encore a 6-8 rad/s (mesure
// sim). Trop serres, l'agent ne touche jamais le bonus et n'apprend rien.
constexpr float QL_UP_RAD   = 0.175f;  // ~10 deg : "en haut"
constexpr float QL_UP_TDOT  = 8.0f;    // rad/s max du bras pour toucher QL_R_UP
constexpr float QL_R_UP     = 5.0f;
constexpr float QL_BAL_RAD  = 0.087f;  // ~5 deg : "equilibre"
constexpr float QL_BAL_ADOT = 1.0f;    // rad/s max du pendule
constexpr float QL_BAL_TDOT = 5.0f;    // rad/s max du bras
constexpr float QL_R_BAL    = 20.0f;
// Poids du shaping POTENTIEL sur l'energie (0 = desactive). Voir reward() dans
// qlearning.cpp : sans lui l'agent se fige a l'horizontale, et les bonus de
// sommet ci-dessus ne peuvent pas l'en sortir puisqu'il ne les echantillonne
// jamais meme a 50 Hz (le pendule traverse +/-10 deg en ~9 ms en balistique).
// /!\ Le terme DOIT rester sous forme POTENTIELLE : gamma*Phi(s') - Phi(s) avec
// Phi = -QL_K_ENERGY*|E - eTop|/(2*eTop). Un simple bonus d'etat n'est pas
// policy-invariant (Ng, Harada & Russell 1999) et l'agent l'accumule en RESTANT
// dans les bons etats : mesure en simu, ca creait un nouvel optimum local ou le
// pendule TOURNE indefiniment a E = eTop (7500 de recompense par episode, zero
// equilibre). Sous forme de difference le terme se telescope a zero sur toute
// trajectoire fermee : tourner en rond ne paie plus.
constexpr float QL_K_ENERGY = 8.0f;
// Shaping POTENTIEL d'approche du sommet. Contrairement au cône QL_K_BAL
// (récompense de maintien), ce terme paie une TRANSITION vers un état haut et
// lent : F = gamma*PhiApproche(s') - PhiApproche(s). Il guide donc le freinage
// avant le rattrapage sans rendre rentable une orbite qui repasse sans cesse au
// sommet, puisque le gain se télescope sur toute trajectoire fermée.
constexpr float QL_K_APPROACH    = 20.0f; // poids du potentiel
constexpr float QL_APPROACH_ADOT = 8.0f;  // rad/s, échelle de vitesse pendule
constexpr float QL_APPROACH_TDOT = 8.0f;  // rad/s, échelle de vitesse bras
// Traces d'eligibilite de SARSA(lambda). Elles sont creuses, remplacantes et
// bornees par QL_TRACE_MAX : aucun second tableau dense n'est alloue.
constexpr float QL_LAMBDA   = 0.92f;

// --- Tile coding partage firmware/simulation (cf. sim/tiles.py) ---
// TC_TILINGS = 0 -> Q-table classique. Sinon, la fonction de valeur devient une
// somme de poids sur TC_TILINGS quadrillages decales : deux etats voisins
// partagent la plupart de leurs paves, donc apprendre sur l'un generalise a
// l'autre. C'est ce qui permet d'avoir a la fois une resolution fine et une
// dimension theta_dot sans exploser le nombre d'echantillons necessaires (une
// table, elle, traite chaque case comme independante — mesure : chaque ajout de
// resolution RALENTISSAIT l'apprentissage).
// Le format SPL1 est lu et ecrit a l'identique par storage.cpp.
constexpr int   TC_TILINGS = 0;
constexpr int   TC_N_ALPHA = 12;   // paves par quadrillage sur alpha (circulaire)
constexpr int   TC_N_ADOT  = 12;   // sur alpha_dot
constexpr int   TC_N_TDOT  = 0;    // sur theta_dot (0 = pas dans l'etat)
// Hachage des paves dans une table de 2^TC_HASH_BITS traits (0 = indexation
// directe). C'est la condition pour tenir sur la Teensy : en indexation directe
// la config validee en simulation demande 4,64 Mo pour 512 ko de DMAMEM.
// Les collisions sont benignes tant que la table reste grande devant le nombre
// d'etats REELLEMENT visites (le pendule ne parcourt qu'une variete etroite de
// l'espace d'etat, pas tout le produit cartesien).
constexpr int   TC_HASH_BITS = 0;
// Deformation de l'axe alpha avant pavage : u = signe(a)*pi*(|a|/pi)^p.
// A p < 1 la zone proche de la verticale est ETIREE, donc couverte par plus de
// paves, sans en ajouter ailleurs. L'equilibre exige ~0,5 deg de resolution mais
// seulement sur +/-10 deg, alors que le swing-up se contente de quelques degres
// sur 360 : une grille uniforme paie la finesse partout, gonfle le nombre
// d'indices distincts et fait exploser les collisions du hachage.
// 1.0 = pavage uniforme (historique).
constexpr float TC_ALPHA_WARP = 1.0f;

// --- Tile coding factorise, portable dans les 512 kio de RAM2 ---
// Une seule fonction de valeur SARSA, mais deux banques de traits sans
// collision :
//   - banque GLOBALE 3D grossiere (alpha, alpha_dot, theta_dot) hors du
//     voisinage du sommet : theta_dot reste utile pour planifier l'approche ;
//   - banque LOCALE 3D (alpha, alpha_dot, theta_dot) pres du sommet :
//     resolution fine la ou theta_dot est indispensable a l'equilibre.
// Le gate ne choisit pas un autre controleur : il choisit seulement les traits
// actifs du meme agent SARSA. Les transitions a travers la frontiere sont
// bootstrappees normalement.
// Empreinte par defaut :
//   global = 8 * 12 * (18+1) * (5+1) = 10 944 traits
//   local  = 8 * (16+1) * (6+1) * (5+1) = 5 712 traits
//   poids  = 16 656 * 7 actions * 4 octets = 455,4 kio.
constexpr int   TC_SPLIT             = 1;
constexpr int   TC_GLOBAL_TILINGS    = 8;
constexpr int   TC_GLOBAL_N_ALPHA    = 12;
constexpr int   TC_GLOBAL_N_ADOT     = 18;
constexpr int   TC_GLOBAL_N_TDOT     = 5;
constexpr float TC_GLOBAL_LR_SCALE   = 0.333333f;
constexpr int   TC_LOCAL_TILINGS     = 8;
constexpr float TC_LOCAL_RAD         = 0.35f;
constexpr float TC_LOCAL_ADOT_MAX    = 8.0f;
constexpr float TC_LOCAL_TDOT_MAX    = 8.0f;
constexpr int   TC_LOCAL_N_ALPHA     = 16;
constexpr int   TC_LOCAL_N_ADOT      = 6;
constexpr int   TC_LOCAL_N_TDOT      = 5;
// Les visites pres du sommet sont rares pendant un entrainement qui part
// toujours du bas. Ce multiplicateur permet a la banque locale d'apprendre plus
// vite sans rendre agressives les mises a jour du swing-up.
constexpr float TC_LOCAL_LR_SCALE    = 2.0f;

// 0 : gate dur, seule une banque est active. 1 : pres du sommet, la banque
// locale apprend un residu par-dessus les traits globaux (memoire inchangee).
constexpr int   TC_SPLIT_OVERLAP      = 0;

// Traces d'eligibilite : coupure et PLAFOND. Le nombre de traces actives vaut
// ~n_tilings * ln(seuil)/ln(gamma*lambda) : a lambda=0.92 et 200 Hz ca fait
// ~870 traces parcourues a chaque pas RL. Un tampon non borne est inacceptable
// sur Teensy, et cote simulation c'etait le goulot (10,5 milliards d'operations
// pour un balayage). Une trace decroit avec son age, donc plafonner revient a
// jeter les plus faibles.
constexpr float QL_TRACE_MIN = 0.01f;  // en dessous, la trace est oubliee
constexpr int   QL_TRACE_MAX = 512;    // nombre max de traces vivantes

// Amorcage du Q-learning (0 = desactive). Meme role que SWING_KICK_* pour la loi
// classique : les deux lois valent EXACTEMENT zero au repos en bas, donc rien ne
// demarre. En entrainement c'est epsilon qui cassait la symetrie ; en glouton
// plus rien, et l'agent reste indefiniment immobile en bas (mesure : la valeur
// de l'action neutre y est 10,4 contre ~1 pour toutes les autres).
constexpr float QL_KICK = 1.0f;
// Malus de vitesse du pendule PRES DU HAUT seulement (jamais ailleurs : un
// swing-up a besoin de cette vitesse, cf. invariants). Doit etre du meme ordre
// que QL_K_ENERGY, c'est lui qui rend "s'arreter en haut" strictement meilleur
// que "traverser le haut a fond".
constexpr float QL_ADOT_TOP_RAD = 0.80f;  // rad : zone ou le malus s'applique
constexpr float QL_K_ADOT_TOP   = 0.06f;  // par (rad/s)
// Dérive du bras près du sommet. Le gate des bonus à 5/8 rad/s retire seulement
// une récompense ; il ne rend pas une accélération continue franchement
// mauvaise. Ce coût quadratique est local au sommet et ne gêne donc jamais le
// pompage. 0 = comportement historique.
constexpr float QL_K_TDOT_TOP   = 0.10f;  // par (rad/s)^2
constexpr float QL_TDOT_TOP_RAD = 0.35f;  // rad : zone locale du coût

// Cone de recompense pres du haut : le GRADIENT qui manquait a l'equilibre.
// 1+cos(alpha) est un extremum en 0, donc PLAT au second ordre : sur +/-8 deg il
// ne varie que de 0,011 pour une base de 2 (0,5 %), noye sous le cout d'effort
// (0,02). Rien n'indiquait donc a l'agent quelle action GARDE le pendule en haut.
// Mesure : 0,10 s de tenue meme avec 70 % des episodes dedies a l'equilibre,
// alors que la loi classique quantifiee sur les MEMES 7 actions discretes a
// 200 Hz tient 29,4 s — c'est-a-dire que la cible etait representable mais pas
// apprenable. Forme en cone (max au sommet, nul au bord) pour rester NON
// NEGATIVE, cf. invariant de reward().
// Exploration SELON L'ETAT : rafales longues en bas (pomper exige des poussees
// soutenues), courtes pres du haut (une action aleatoire tenue 200 ms fait
// tomber le pendule, qui bascule en ~100 ms). Mesure a 200 Hz, tenue moyenne :
// rafales 200 ms -> 0,17 s ; 40 ms -> 0,45 s ; 5 ms -> 0,02 s ET swing-up
// detruit. Un reglage unique ne peut que faire un compromis entre les deux.
// QL_EXPLORE_NEAR_RAD = 0 desactive (rafale unique, comportement historique).
// SARSA(lambda) au lieu de Q(lambda) (0 = Q-learning, historique). Q-learning
// bootstrappe sur max_a Q(s',a), c'est-a-dire la valeur de la politique
// GLOUTONNE, alors que l'agent se comporte en epsilon-glouton. Pres d'un
// equilibre instable l'ecart est brutal : la politique gloutonne tiendrait
// indefiniment (valeur enorme) tandis que le comportement reel fait tomber le
// pendule en permanence. La cible est donc systematiquement optimiste et l'agent
// n'apprend jamais que SA conduite perd. SARSA apprend la valeur de la politique
// suivie, exploration comprise — c'est le cas d'ecole du "cliff walking".
constexpr float QL_SARSA = 1.0f;

constexpr float QL_EXPLORE_NEAR_RAD = 0.35f; // rad : zone locale
constexpr int   QL_EXPLORE_HOLD_TOP = 1;     // pas RL de rafale pres du haut
// Plafond d'epsilon dans cette même zone. À epsilon=0,02 et 200 Hz, quatre
// actions aléatoires par seconde empêchent l'agent de vivre une tenue longue,
// donc SARSA n'a aucun retour stable à propager. Valeur < 0 : désactivé.
constexpr float QL_EXPLORE_EPS_TOP  = 0.30f; // repli si QL_EPS_TOP0 < 0

constexpr float QL_K_BAL        = 10.0f;
constexpr float QL_BAL_CONE_RAD  = 0.25f; // rad : demi-largeur du cone
constexpr float QL_BAL_CONE_ADOT = 3.0f;  // rad/s : idem sur la vitesse
// Le bras doit lui aussi ralentir avant le rattrapage. Sans ce terme, deux
// états ayant le même (alpha, alpha_dot) reçoivent le même shaping même si l'un
// arrive avec un bras presque immobile et l'autre à 8 rad/s. Le facteur reste
// positif et s'annule progressivement au seuil du bonus QL_R_UP.
constexpr float QL_BAL_CONE_TDOT = 8.0f;  // rad/s : extinction du cône (0 = off)

// --- Pause entre deux episodes : MOTEUR COUPE, on attend le repos ---
// On ne ramene PAS le bras a theta = 0 : theta n'est pas dans l'etat du RL
// (l'etat est [alpha, alpha_dot]), donc sa position n'apporte aucune coherence
// a l'apprentissage, et le collecteur tournant l'autorise n'importe ou.
// Aucun couple n'est pilote pendant la pause -> aucune dependance au signe et
// aucun emballement possible. On attend juste que tout s'immobilise, pour que
// chaque episode reparte du meme etat : pendule en bas, au repos.
constexpr float QL_SETTLE_RAD   = 0.35f;  // rad autour de +/-pi = "pendule en bas"
constexpr float QL_SETTLE_ADOT  = 0.60f;  // rad/s = "pendule immobile"
constexpr float QL_SETTLE_TDOT  = 1.00f;  // rad/s = "bras immobile"
constexpr float QL_SETTLE_MAX_S = 10.0f;  // delai max d'attente

// ---------- Machine à états ----------
enum SysState : uint8_t {
  ST_IDLE = 0,
  ST_CLASSIC,        // swing-up + équilibre
  ST_BALANCE_ONLY,   // équilibre seul (placer le pendule en haut à la main)
  ST_QL_TRAIN,       // Q-learning epsilon-greedy
  ST_QL_GREEDY,      // Q-learning exploitation pure
  ST_MOTOR_TEST,     // jog manuel pour vérifier les signes
  ST_MOTOR_AUTOTEST, // séquence auto : basse vitesse + puis -
  ST_MOTOR_OPENLOOP, // test moteur SANS capteur (banc, avant montage)
  ST_DEBUG_ANGLES,   // affichage live des encodeurs, moteur coupé
  ST_SETTINGS,       // éditeur de réglages (moteur coupé)
  ST_FAULT
};

// Test open-loop (menu Debug) : fait tourner le moteur SANS capteur, comme un
// pas-a-pas. Sert a valider moteur/driver/cablage avant le montage mecanique.
// /!\ En open-loop le courant est constant quelle que soit la charge -> ca
// chauffe. Garder la tension basse et les essais courts.
constexpr float OPENLOOP_VOLTAGE  = 4.0f;   // V (fixe)
constexpr float OPENLOOP_VEL_STEP = 2.0f;   // rad/s par cran d'encodeur
constexpr float OPENLOOP_VEL_MAX  = 30.0f;  // rad/s

// Test moteur automatique (menu Debug)
constexpr float    AUTOTEST_DUTY    = 0.12f;
constexpr uint32_t AUTOTEST_RUN_MS  = 2000;
constexpr uint32_t AUTOTEST_STOP_MS = 800;

enum FaultCode : uint8_t {
  FAULT_NONE = 0,
  FAULT_ALPHA_DOT,
  FAULT_THETA_DOT,
  FAULT_THETA_RANGE,
  FAULT_SATURATION,
  FAULT_USER_STOP
};

// État partagé boucle de contrôle <-> reste du programme
struct PendulumState {
  float theta;      // rad, angle du bras (cumulé, non borné)
  float alpha;      // rad, angle du pendule dans [-pi, pi], 0 = haut
  float thetaDot;   // rad/s (filtré)
  float alphaDot;   // rad/s (filtré)
  float duty;       // commande moteur appliquée [-1, 1]
};
