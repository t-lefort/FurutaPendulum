#include "motor.h"
#include "settings.h"
#include "encoders.h"
#include <SimpleFOC.h>
using Settings::cfg;

// ---- Capteur FOC : adaptateur sur l'encodeur du bras ----
// Pas de capteur dédié : moteur et encodeur bras tournent au même rythme (1:1).
// On passe par Encoders:: plutôt que d'instancier un second objet Encoder, pour
// ne pas attacher deux jeux d'interruptions sur les mêmes broches 0/1.
// Lecture d'un compteur = très rapide (aucun I2C), donc loopFOC() est léger.
class ArmShaftSensor : public Sensor {
 public:
  // Sensor::init() est 'protected' dans SimpleFOC : on le republie en public
  // pour pouvoir l'appeler depuis Motor::begin().
  using Sensor::init;

  float getSensorAngle() override {
    float a = fmodf(Encoders::motorShaftAngle(), (float)TWO_PI);
    if (a < 0) a += (float)TWO_PI;      // SimpleFOC attend [0, 2pi)
    return a;
  }
};

// ---- Objets SimpleFOC ----
static BLDCMotor      motor(MOTOR_POLE_PAIRS);
static BLDCDriver3PWM driver(PIN_DRV_PWM_A, PIN_DRV_PWM_B,
                             PIN_DRV_PWM_C, PIN_DRV_EN);
static ArmShaftSensor sensor;

// ---- Etat partage ISR (ecrivain) <-> loop (lecteur) ----
// Chaque champ est un float/bool aligne : ecriture atomique sur Cortex-M7,
// donc pas de section critique necessaire.
static volatile float appliedNorm = 0.0f;  // couple normalise [-1, 1]
static volatile bool  motorOn     = false;
static bool           driverOk    = false; // etage de puissance pret
static bool           focOk       = false; // + alignement capteur reussi

// Open-loop : ne depend QUE de driverOk (pas du capteur).
static volatile bool  openLoop    = false;
static volatile float openLoopVel = 0.0f;  // rad/s

void Motor::begin() {
  sensor.init();                  // Encoders::begin() doit avoir tourne avant
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.pwm_frequency        = FOC_PWM_FREQ_HZ;
  if (!driver.init()) { driverOk = false; focOk = false; return; }
  driverOk = true;
  motor.linkDriver(&driver);

  // Commande en COUPLE (mode tension) : aucune boucle de vitesse/position
  // interne a SimpleFOC — c'est notre controle 1 kHz qui decide de tout.
  motor.controller        = MotionControlType::torque;
  motor.torque_controller = TorqueControlType::voltage;
  motor.voltage_limit     = MOTOR_VOLT_LIMIT;

  // Monitoring : initFOC() imprime le detail de l'alignement sur le port serie,
  // dont une ESTIMATION DES PAIRES DE POLES ("PP check: fail - estimated pp:").
  // C'est le moyen le plus simple de verifier MOTOR_POLE_PAIRS.
  motor.useMonitoring(Serial);

  motor.init();
  focOk = (motor.initFOC() == 1);  // /!\ fait BOUGER le moteur (alignement)
  motor.move(0.0f);
  motorOn = false;

  if (!focOk)
    Serial.println("FOC: echec alignement (encodeur bras ? paires de poles ?)");
}

bool Motor::ready() { return focOk; }

// ---- Appele a 1 kHz depuis l'ISR : aucune E/S, juste la consigne ----
void Motor::setDuty(float u) {
  u = constrain(u, -cfg.dutyLimit, cfg.dutyLimit);
  const float maxStep = cfg.dutySlew * CTRL_DT;
  appliedNorm += constrain(u - appliedNorm, -maxStep, maxStep);
  motorOn = true;
}

void Motor::hardStop() {
  appliedNorm = 0.0f;
  openLoopVel = 0.0f;
  motorOn     = false;      // spin() appliquera un couple nul (roue libre)
}

// ---- Open-loop (banc) ----
void Motor::openLoopEnable(bool on) {
  if (!driverOk) return;
  // Reconfiguration de SimpleFOC : doit etre atomique vis-a-vis du timer FOC
  // qui appelle move()/loopFOC() en parallele.
  noInterrupts();
  openLoop    = on;
  openLoopVel = 0.0f;
  appliedNorm = 0.0f;
  motorOn     = false;
  // En open-loop la tension est fixe et le courant permanent -> on la reduit.
  motor.voltage_limit = on ? OPENLOOP_VOLTAGE : MOTOR_VOLT_LIMIT;
  motor.controller    = on ? MotionControlType::velocity_openloop
                           : MotionControlType::torque;
  // /!\ SimpleFOC appelle disable() quand initFOC() echoue, et move() sort
  // aussitot si le moteur est desactive. Sans ce enable(), l'open-loop serait
  // mort des que l'alignement rate — or c'est precisement le cas ou il sert.
  // Sans danger : spin() n'applique rien en mode couple tant que focOk est faux.
  motor.enable();
  interrupts();
}

void Motor::openLoopSetVelocity(float w) {
  openLoopVel = constrain(w, -OPENLOOP_VEL_MAX, OPENLOOP_VEL_MAX);
  motorOn     = true;
}

// ---- Appele depuis loop(), le plus souvent possible ----
void Motor::spin() {
  // Open-loop n'a besoin que de l'etage de puissance : il doit marcher meme
  // si l'alignement FOC a echoue (c'est tout l'interet du test au banc).
  if (openLoop) {
    if (!driverOk) return;
    motor.loopFOC();                    // no-op en mode open-loop
    motor.move(motorOn ? openLoopVel : 0.0f);
    return;
  }
  if (!focOk) return;
  motor.loopFOC();                      // commutation (lit l'encodeur bras)
  motor.move(motorOn ? appliedNorm * MOTOR_VOLT_LIMIT : 0.0f);
}

float Motor::duty() { return appliedNorm; }
bool  Motor::isSaturated() { return fabsf(appliedNorm) >= cfg.dutyLimit - 1e-4f; }
