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
constexpr float PEND_SIGN = +1.0f;

// ---------- Moteur BLDC / FOC ----------
// Moteur gimbal GBM2804 : 12N14P -> 14 aimants -> 7 paires de poles.
// A CORRIGER si tu changes de moteur (une valeur fausse empeche l'alignement FOC).
constexpr int   MOTOR_POLE_PAIRS = 7;
constexpr float SUPPLY_VOLTAGE   = 15.0f;   // alim du SimpleFOCMini
// Tension q max appliquee au moteur. Sur un GBM2804 (~10 ohm) : 6 V -> 0,6 A,
// tres large sous les 3 A du driver. Monter prudemment si le couple manque.
constexpr float MOTOR_VOLT_LIMIT = 6.0f;
constexpr float FOC_PWM_FREQ_HZ  = 25000.0f; // inaudible
// Frequence de la commutation FOC (timer dedie). Doit etre BIEN plus rapide que
// la boucle de controle : c'est elle qui fabrique les tensions de phase.
// Trop lente -> commutation en escalier = moteur saccade.
constexpr float FOC_FREQ_HZ      = 10000.0f;

// Limites de commande. La "duty" est desormais un COUPLE NORMALISE [-1, 1]
// qui est mis a l'echelle par MOTOR_VOLT_LIMIT.
constexpr float DUTY_LIMIT      = 0.90f;  // fraction max du couple dispo
constexpr float DUTY_SLEW_PER_S = 20.0f;   // variation max par seconde
// (La compensation de zone morte a ete supprimee : la FOC est lisse des
//  0 tr/min. Le frottement statique de la mecanique est traite par le terme
//  integral K_TH_I plus bas.)

// ---------- Boucles ----------
constexpr float    CTRL_FREQ_HZ = 1000.0f;          // boucle de contrôle
constexpr float    CTRL_DT      = 1.0f / CTRL_FREQ_HZ;
constexpr uint32_t RL_DIVIDER   = 20;               // Q-learning à 1000/20 = 50 Hz
constexpr float    RL_DT        = CTRL_DT * RL_DIVIDER;
constexpr float    VEL_FILT_ALPHA = 0.20f;          // passe-bas vitesses (0..1, plus grand = moins filtré)

// ---------- Sécurité ----------
constexpr float ALPHA_DOT_MAX   = 30.0f;  // rad/s — coupure si dépassé
constexpr float THETA_DOT_MAX   = 45.0f;  // rad/s
constexpr float THETA_TURNS_MAX = 10.0f;   // tours max du bras (0 = illimité, si collecteur tournant)
constexpr float SAT_TIMEOUT_S   = 8.0f;   // coupure si duty saturé en continu trop longtemps

// ---------- Modèle physique (pour le swing-up énergie) ----------
// À ajuster avec tes valeurs réelles (masse pendule, distance pivot -> centre de masse)
constexpr float PEND_MASS  = 0.080f;                 // kg
constexpr float PEND_LCOM  = 0.080f;                 // m (pivot -> centre de masse)
constexpr float PEND_LEN   = 0.115f;                 // m (longueur totale)
constexpr float G_GRAV     = 9.81f;
// Énergie cible E_TOP et inertie PEND_J sont dérivées de la masse/longueur :
// elles vivent maintenant dans Settings (cfg.eTop() / cfg.pendJ()) car ces
// paramètres sont réglables au menu.

// ---------- Gains mode Classic (POINTS DE DÉPART, à régler) ----------
// Swing-up : u = KE_SWING * (E - E_TOP) * alpha_dot * cos(alpha) - KTHD_SWING * theta_dot
constexpr float KE_SWING   = -50.0f;
constexpr float KTHD_SWING = 0.004f;

// Équilibre (retour d'état, sortie = duty) :
// u = -(K_ALPHA*alpha + K_ADOT*alpha_dot + K_TH*theta + K_THD*theta_dot)
constexpr float K_ALPHA = 9.0f;
constexpr float K_ADOT  = 0.60f;
constexpr float K_TH    = 0.20f;   // 0 pour ignorer theta (collecteur tournant)
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

// ---------- PID vitesse bras (boucle interne pour le Q-learning) ----------
constexpr float KP_VEL = 0.030f;   // duty / (rad/s)
constexpr float KI_VEL = 0.080f;
constexpr float VEL_I_MAX = 0.30f; // anti-windup

// ---------- Q-learning ----------
constexpr int   QL_N_ALPHA  = 49;
constexpr int   QL_N_ADOT   = 31;
constexpr int   QL_N_ACT    = 7;
constexpr float QL_ADOT_MAX = 20.0f;   // rad/s, plage de discrétisation
constexpr float QL_W_MAX    = 12.0f;   // rad/s, vitesse bras max commandée
constexpr float QL_LR       = 0.05f;   // learning rate
constexpr float QL_GAMMA    = 0.97f;
constexpr float QL_EPS0     = 0.30f;
constexpr float QL_EPS_MIN  = 0.02f;
constexpr float QL_EPS_DECAY = 0.995f; // par épisode
constexpr float QL_EPISODE_S = 15.0f;  // durée d'un épisode

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
