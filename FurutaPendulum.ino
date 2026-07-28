// ============================================================
//  Pendule de Furuta — Teensy 4.1 + BLDC gimbal (FOC) + SimpleFOCMini + GC9A01
//  Modes : Classic (swing-up + équilibre) et Q-learning embarqué
//
//  Boucle de contrôle : 1 kHz (IntervalTimer)
//  Q-learning         : 50 Hz (dans la boucle de contrôle)
//  Écran              : 10 Hz  (loop)
//  Logs SD            : 50 Hz  (loop)
// ============================================================
#include "config.h"
#include "settings.h"
#include "encoders.h"
#include "motor.h"
#include "safety.h"
#include "control_classic.h"
#include "qlearning.h"
#include "storage.h"
#include "ui.h"

using Settings::cfg;

static IntervalTimer ctrlTimer;
static IntervalTimer focTimer;   // commutation FOC, bien plus rapide

// --- État partagé (écrit par l'ISR 1 kHz, lu par loop) ---
static volatile SysState  sysState = ST_IDLE;
static volatile FaultCode faultCode = FAULT_NONE;
static PendulumState      isrState;         // usage exclusif ISR
static volatile bool      snapReady = false;
static PendulumState      snapshot;         // copie pour loop

// Q-learning
static uint32_t rlCounter = 0;
static float    rlUCmd = 0.0f;    // action brute choisie par l'agent
static float    rlApplied = 0.0f; // idem, lissée (QL_U_TAU) avant le moteur
static volatile bool qlSaveRequest = false;

// Jog manuel
static volatile float jogDuty = 0.0f;

// Test open-loop (consigne de vitesse arbre moteur, rad/s)
static volatile float olVel = 0.0f;

// Test moteur automatique
static volatile uint8_t  atPhase = 0;
static uint32_t          atTicks = 0;
static float             atTheta0 = 0.0f;
static volatile float    atDeltaPlus = 0.0f, atDeltaMinus = 0.0f;

// ------------------------------------------------------------
//  Boucle de contrôle 1 kHz
// ------------------------------------------------------------
static void controlTick() {
  Encoders::update(isrState);

  const SysState st = sysState;
  const bool motorActive =
      (st == ST_CLASSIC || st == ST_BALANCE_ONLY || st == ST_QL_TRAIN ||
       st == ST_QL_GREEDY || st == ST_MOTOR_TEST || st == ST_MOTOR_AUTOTEST ||
       st == ST_MOTOR_OPENLOOP);

  if (motorActive) {
    const FaultCode f = Safety::check(isrState, Motor::isSaturated());
    if (f != FAULT_NONE) {
      Motor::hardStop();
      faultCode = f;
      sysState = ST_FAULT;
      isrState.duty = 0.0f;
      return;
    }
  }

  switch (st) {
    case ST_CLASSIC:
    case ST_BALANCE_ONLY:
      Motor::setDuty(ControlClassic::update(isrState));
      break;

    case ST_QL_TRAIN:
    case ST_QL_GREEDY:
      if (++rlCounter >= RL_DIVIDER) {
        rlCounter = 0;
        bool epEnd = false;
        rlUCmd = QLearning::step(isrState, epEnd);
        if (epEnd) qlSaveRequest = true;
      }
      if (QLearning::isPaused()) {
        // Pause entre deux épisodes : moteur COUPÉ, tout retombe tout seul.
        // Aucun couple piloté -> aucun emballement possible ; theta est
        // re-zéroté par QLearning au démarrage de l'épisode suivant.
        Motor::hardStop();
        rlApplied = 0.0f;
      } else {
        // L'action du RL EST le couple (aucune boucle intermédiaire), mais on
        // arrondit les fronts : les actions sont discrètes et peuvent
        // s'inverser d'un pas à l'autre, ce qui ferait claquer la mécanique.
        rlApplied += (rlUCmd - rlApplied) * (CTRL_DT / QL_U_TAU);
        Motor::setDuty(rlApplied, QL_DUTY_SLEW_PER_S);
      }
      break;

    case ST_MOTOR_TEST:
      Motor::setDuty(jogDuty);
      break;

    case ST_MOTOR_OPENLOOP:
      Motor::openLoopSetVelocity(olVel);
      break;

    case ST_MOTOR_AUTOTEST:
      atTicks++;
      switch (atPhase) {
        case 0:  // mémorise theta de départ
          atTheta0 = isrState.theta;
          atPhase = 1; atTicks = 0;
          break;
        case 1:  // sens +
          Motor::setDuty(AUTOTEST_DUTY);
          if (atTicks >= AUTOTEST_RUN_MS) {
            atDeltaPlus = isrState.theta - atTheta0;
            atPhase = 2; atTicks = 0;
          }
          break;
        case 2:  // pause
          Motor::setDuty(0.0f);
          if (atTicks >= AUTOTEST_STOP_MS) {
            atTheta0 = isrState.theta;
            atPhase = 3; atTicks = 0;
          }
          break;
        case 3:  // sens -
          Motor::setDuty(-AUTOTEST_DUTY);
          if (atTicks >= AUTOTEST_RUN_MS) {
            atDeltaMinus = isrState.theta - atTheta0;
            atPhase = 4;
          }
          break;
        default: // terminé
          Motor::hardStop();
          break;
      }
      break;

    default:  // IDLE, DEBUG_ANGLES, FAULT : moteur coupé
      Motor::hardStop();
      break;
  }

  isrState.duty = Motor::duty();
  if (!snapReady) {
    snapshot = isrState;
    snapReady = true;
  }
}

// ------------------------------------------------------------
//  Transitions (depuis loop uniquement)
// ------------------------------------------------------------
static void enterState(SysState next) {
  noInterrupts();
  Motor::hardStop();
  Safety::reset();
  rlCounter = 0;
  rlUCmd = 0.0f;
  rlApplied = 0.0f;
  jogDuty = 0.0f;
  olVel   = 0.0f;
  atPhase = 0; atTicks = 0;
  atDeltaPlus = 0.0f; atDeltaMinus = 0.0f;

  switch (next) {
    case ST_CLASSIC:      ControlClassic::reset(false);   break;
    case ST_BALANCE_ONLY: ControlClassic::reset(true);    break;
    case ST_QL_TRAIN:     QLearning::startSession(false); break;
    case ST_QL_GREEDY:    QLearning::startSession(true);  break;
    case ST_SETTINGS:     UI::settingsReset();            break;
    default: break;
  }
  // Le mode open-loop change le mode de commande SimpleFOC : on l'arme/désarme
  // à chaque transition pour ne jamais y rester coincé.
  Motor::openLoopEnable(next == ST_MOTOR_OPENLOOP);
  sysState = next;
  interrupts();
  UI::invalidate();

  if (next == ST_QL_TRAIN) Storage::logStart();
  else                     Storage::logStop();
}

static void stopToIdle() {
  if (sysState == ST_QL_TRAIN || sysState == ST_QL_GREEDY) {
    noInterrupts();
    QLearning::endEpisode();
    interrupts();
  }
  enterState(ST_IDLE);
}

static void flashMessage(const char *msg) {
  UI::drawMessage(msg);
  delay(700);
  UI::invalidate();
}

static bool saveAgent(const char *weightsPath, const char *statePath) {
  QLearning::PersistState progress;
  QLearning::getPersistState(progress);
  const uint32_t checksum =
      Storage::qChecksum(QLearning::table(), QLearning::tableCount());
  if (!Storage::saveQTable(
          weightsPath, QLearning::table(), QLearning::tableCount()))
    return false;
  return Storage::saveQLState(statePath, progress, checksum);
}

static bool loadAgent(const char *weightsPath, const char *statePath) {
  if (!Storage::loadQTable(
          weightsPath, QLearning::table(), QLearning::tableCount()))
    return false;

  const uint32_t checksum =
      Storage::qChecksum(QLearning::table(), QLearning::tableCount());
  QLearning::PersistState progress;
  if (!Storage::loadQLState(statePath, progress, checksum) ||
      !QLearning::restoreTrainingState(progress)) {
    // Une politique SPL1 issue de la simulation n'a pas de sidecar : elle
    // repart alors avec le planning epsilon/LR du profil robuste.
    QLearning::resetTrainingState();
  }
  return true;
}

static void handleMenuSelect() {
  const UI::MenuId m = UI::currentMenu();
  const int i = UI::menuIndex();

  if (m == UI::M_MAIN) {
    switch (i) {
      case 0: UI::setMenu(UI::M_CLASSIC); break;
      case 1: UI::setMenu(UI::M_QL);      break;
      case 2: UI::setMenu(UI::M_DEBUG);   break;
      case 3: enterState(ST_SETTINGS);    break;
      case 4:  // Recalibration : pendule immobile en bas !
        noInterrupts();
        Encoders::calibrateBottom();
        interrupts();
        flashMessage("Calibre !");
        break;
    }
  } else if (m == UI::M_CLASSIC) {
    switch (i) {
      case 0: UI::setMenu(UI::M_MAIN);     break;
      case 1: enterState(ST_CLASSIC);      break;
      case 2: enterState(ST_BALANCE_ONLY); break;
    }
  } else if (m == UI::M_QL) {
    switch (i) {
      case 0: UI::setMenu(UI::M_MAIN); break;
      case 1: enterState(ST_QL_TRAIN);  break;
      case 2: enterState(ST_QL_GREEDY); break;
      case 3:
        flashMessage(saveAgent("/q_current.bin", "/q_state.bin")
                     ? "Q sauvee" : "Erreur SD");
        break;
      case 4:
        flashMessage(loadAgent("/q_current.bin", "/q_state.bin")
                     ? "Q chargee" : "Erreur SD");
        break;
      case 5:
        noInterrupts();
        QLearning::resetTable();
        QLearning::resetTrainingState();
        interrupts();
        flashMessage("Q resetee");
        break;
    }
  } else if (m == UI::M_DEBUG) {
    switch (i) {
      case 0: UI::setMenu(UI::M_MAIN);       break;
      case 1: enterState(ST_DEBUG_ANGLES);   break;
      case 2: enterState(ST_MOTOR_AUTOTEST); break;
      case 3: enterState(ST_MOTOR_TEST);     break;
      case 4: enterState(ST_MOTOR_OPENLOOP); break;
    }
  }
}

// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);

  Settings::begin();    // charge les réglages EEPROM (ou les défauts) AVANT tout
  UI::begin();               // avant le moteur pour pouvoir prévenir à l'écran
  Encoders::begin();         // init encodeurs : la FOC lit celui du bras
  UI::drawMessage("Alignement...");
  Motor::begin();            // /!\ initFOC() FAIT BOUGER LE MOTEUR (~1-2 s)
  Encoders::calibrateBottom(); // re-zéro APRÈS l'alignement (pendule immobile en bas)
  QLearning::begin();
  Storage::begin();
  // Alignement raté = mode couple inutilisable (Classic/QL). Le menu Debug ->
  // Openloop reste utilisable. On tient le message assez longtemps pour le lire.
  if (!Motor::ready()) { UI::drawMessage("Align. KO"); delay(2000); }

  UI::invalidate();
  // FOC en priorité HAUTE (nombre plus petit = plus prioritaire) : la
  // commutation ne doit jamais être retardée, ni par le contrôle ni par loop().
  focTimer.begin(Motor::spin, (int)(1000000.0f / FOC_FREQ_HZ));
  focTimer.priority(32);
  ctrlTimer.begin(controlTick, (int)(1000000.0f / CTRL_FREQ_HZ));
  ctrlTimer.priority(64);
}

// ------------------------------------------------------------
void loop() {
  static uint32_t lastDraw = 0;
  static uint32_t lastLog  = 0;
  static bool     menuDirty = true;

  // NB : la commutation FOC tourne dans focTimer (10 kHz), PAS ici — loop() se
  // bloque trop longtemps (SPI/SD/Serial) et ferait saccader le moteur.

  // ---- Entrées utilisateur ----
  const UI::Event ev = UI::poll();
  const SysState st = sysState;

  if (ev != UI::EV_NONE) {
    if (st == ST_IDLE) {
      if (ev == UI::EV_UP)   { UI::menuMove(+1); menuDirty = true; }
      if (ev == UI::EV_DOWN) { UI::menuMove(-1); menuDirty = true; }
      if (ev == UI::EV_CLICK) { handleMenuSelect(); menuDirty = true; }
      if (ev == UI::EV_LONG)  { UI::setMenu(UI::M_MAIN); menuDirty = true; }
    } else if (st == ST_FAULT) {
      if (ev == UI::EV_CLICK || ev == UI::EV_LONG) enterState(ST_IDLE);
    } else if (st == ST_MOTOR_TEST) {
      if (ev == UI::EV_UP)   jogDuty = constrain(jogDuty + 0.05f, -cfg.dutyLimit, cfg.dutyLimit);
      if (ev == UI::EV_DOWN) jogDuty = constrain(jogDuty - 0.05f, -cfg.dutyLimit, cfg.dutyLimit);
      if (ev == UI::EV_CLICK || ev == UI::EV_LONG) stopToIdle();
    } else if (st == ST_MOTOR_OPENLOOP) {
      if (ev == UI::EV_UP)
        olVel = constrain(olVel + OPENLOOP_VEL_STEP, -OPENLOOP_VEL_MAX, OPENLOOP_VEL_MAX);
      if (ev == UI::EV_DOWN)
        olVel = constrain(olVel - OPENLOOP_VEL_STEP, -OPENLOOP_VEL_MAX, OPENLOOP_VEL_MAX);
      if (ev == UI::EV_CLICK || ev == UI::EV_LONG) stopToIdle();
    } else if (st == ST_SETTINGS) {
      // Éditeur de réglages : la navigation/édition est gérée dans UI.
      // Renvoie true quand on demande à sortir (appui long ou "< Retour").
      if (UI::settingsInput(ev)) { enterState(ST_IDLE); menuDirty = true; }
    } else {
      // Tout clic en mode actif = ARRÊT (sécurité logicielle)
      if (ev == UI::EV_CLICK || ev == UI::EV_LONG) stopToIdle();
    }
  }

  // ---- Snapshot de l'état ----
  PendulumState s;
  bool haveSnap = false;
  noInterrupts();
  if (snapReady) { s = snapshot; snapReady = false; haveSnap = true; }
  interrupts();
  static PendulumState lastS = {};
  if (haveSnap) lastS = s;

  // ---- Sauvegarde automatique de la Q-table ----
  if (qlSaveRequest) {
    qlSaveRequest = false;
    static float lastBestSaved = -1e9f;
    const auto &qs = QLearning::stats();
    if (qs.bestReward > lastBestSaved) {
      lastBestSaved = qs.bestReward;
      saveAgent("/q_best.bin", "/q_best_state.bin");
    }
    saveAgent("/q_current.bin", "/q_state.bin");
  }

  // ---- Logs (50 Hz) ----
  if (st == ST_QL_TRAIN && millis() - lastLog >= 20) {
    lastLog = millis();
    const auto &qs = QLearning::stats();
    Storage::logRow(millis(), (uint8_t)st, qs.episode, lastS, qs.lastAction,
                    qs.lastStepReward, qs.episodeReward, qs.epsilon,
                    qs.epsilonTop, qs.learningRate);
  }

  // ---- Écran (10 Hz) ----
  static SysState lastDrawnState = ST_FAULT;
  if (st != lastDrawnState) {
    lastDrawnState = st;
    UI::invalidate();
    if (st == ST_IDLE) menuDirty = true;
  }
  if (millis() - lastDraw >= 100) {
    lastDraw = millis();
    switch (st) {
      case ST_IDLE:
        if (menuDirty) { UI::drawMenu(Storage::available()); menuDirty = false; }
        break;
      case ST_CLASSIC:
      case ST_BALANCE_ONLY:
        UI::drawClassic(lastS, (uint8_t)ControlClassic::phase(),
                        st == ST_BALANCE_ONLY);
        break;
      case ST_QL_TRAIN:
      case ST_QL_GREEDY: {
        const auto &qs = QLearning::stats();
        UI::drawQLearn(lastS, qs.episode, qs.epsilon, qs.episodeReward,
                       qs.bestReward, qs.lastAction, st == ST_QL_GREEDY);
        break;
      }
      case ST_MOTOR_TEST:
        UI::drawMotorTest(lastS);
        break;
      case ST_MOTOR_OPENLOOP:
        UI::drawOpenLoop(olVel, lastS);
        break;
      case ST_MOTOR_AUTOTEST:
        UI::drawAutoTest(atPhase, atDeltaPlus, atDeltaMinus);
        break;
      case ST_DEBUG_ANGLES:
        UI::drawDebugAngles(lastS, Encoders::rawArm(), Encoders::rawPend());
        break;
      case ST_SETTINGS:
        UI::drawSettings();
        break;
      case ST_FAULT:
        UI::drawFault((uint8_t)faultCode);
        break;
    }
  }

  // ---- Debug série ----
  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 200) {
    lastPrint = millis();
    // th en TOURS (plus lisible face aux limites) + p=1 pendant la pause
    // inter-episode du Q-learning (moteur coupe).
    Serial.printf("st=%d p=%d th=%.2ftr a=%.3f ad=%.2f td=%.2f u=%.2f\n",
                  (int)st, (int)QLearning::isPaused(),
                  lastS.theta / (float)TWO_PI, lastS.alpha,
                  lastS.alphaDot, lastS.thetaDot, lastS.duty);
  }
}
