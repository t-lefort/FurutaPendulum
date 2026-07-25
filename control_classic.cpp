#include "control_classic.h"
#include "settings.h"
using Settings::cfg;

static ControlClassic::Phase ph = ControlClassic::SWINGUP;
static bool balanceOnlyMode = false;

void ControlClassic::reset(bool balanceOnly) {
  balanceOnlyMode = balanceOnly;
  ph = balanceOnly ? BALANCE : SWINGUP;
}

ControlClassic::Phase ControlClassic::phase() { return ph; }

float ControlClassic::update(const PendulumState &s) {
  // ---- Commutation ----
  if (ph == SWINGUP) {
    if (fabsf(s.alpha) < BAL_ENTER_RAD && fabsf(s.alphaDot) < BAL_ENTER_ADOT)
      ph = BALANCE;
  } else {
    if (fabsf(s.alpha) > BAL_EXIT_RAD)
      ph = balanceOnlyMode ? BALANCE : SWINGUP;  // en balance-only on ne pompe jamais
  }

  if (ph == BALANCE) {
    if (balanceOnlyMode && fabsf(s.alpha) > BAL_EXIT_RAD) return 0.0f; // attend qu'on le place en haut
    // Retour d'état linéaire autour de la verticale (PD étendu / LQR simplifié)
    return -(cfg.kAlpha * s.alpha
           + cfg.kAdot  * s.alphaDot
           + cfg.kTh    * s.theta
           + cfg.kThd   * s.thetaDot);
  }

  // ---- Swing-up par régulation d'énergie ----
  // E = 1/2 J alpha_dot^2 + m g l cos(alpha)   (réf. : E = -mgl en bas, +mgl en haut)
  const float eTop = cfg.eTop();
  const float E = 0.5f * cfg.pendJ() * s.alphaDot * s.alphaDot
                + eTop * cosf(s.alpha);
  float u = cfg.keSwing * (E - eTop) * s.alphaDot * cosf(s.alpha)
          - cfg.kthdSwing * s.thetaDot;
  // Note : selon les conventions de signes réelles (ARM_SIGN/PEND_SIGN et sens
  // de montage), il peut falloir inverser : u = -u. Le test : le pendule doit
  // osciller DE PLUS EN PLUS haut. S'il s'amortit, inverse le signe de KE_SWING.
  return u;
}
