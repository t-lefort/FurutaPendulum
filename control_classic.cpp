#include "control_classic.h"
#include "settings.h"
using Settings::cfg;

static ControlClassic::Phase ph = ControlClassic::SWINGUP;
static bool balanceOnlyMode = false;
static float thetaInt = 0.0f;   // intégrale de theta (anti-frottement statique)

void ControlClassic::reset(bool balanceOnly) {
  balanceOnlyMode = balanceOnly;
  ph = balanceOnly ? BALANCE : SWINGUP;
  thetaInt = 0.0f;
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
    if (balanceOnlyMode && fabsf(s.alpha) > BAL_EXIT_RAD) {  // attend qu'on le place en haut
      thetaInt = 0.0f;
      return 0.0f;
    }

    // Terme intégral sur theta : le frottement statique du train d'engrenages
    // crée un seuil de décollement. Sous ce seuil, la part proportionnelle
    // K_th*theta ne suffit plus et le bras reste planté loin de 0 ; l'intégrale
    // monte alors lentement jusqu'à le débloquer.
    // On n'intègre QUE près de la verticale, sinon elle se charge pendant les
    // transitoires (arrivée de swing-up) et provoque un à-coup.
    float iTerm = 0.0f;
    if (cfg.kThi > 0.0f && fabsf(s.alpha) < BAL_ENTER_RAD) {
      // Bras revenu à la maison ET immobile : plus rien à débloquer, on
      // désactive l'intégrale. Décharge progressive (constante de temps
      // TH_I_FADE_S) et non brutale : une coupure nette ferait un saut de
      // commande pouvant atteindre TH_I_MAX, qui secouerait le pendule.
      const bool atHome = fabsf(s.theta)    < TH_I_DEAD_RAD &&
                          fabsf(s.thetaDot) < TH_I_DEAD_DOT;
      if (atHome) thetaInt -= thetaInt * (CTRL_DT / TH_I_FADE_S);
      else        thetaInt += s.theta * CTRL_DT;

      iTerm = cfg.kThi * thetaInt;
      // Anti-windup par back-calculation : on borne la CONTRIBUTION à la
      // commande, et on recale l'intégrale en conséquence (pas de charge
      // fantôme qui mettrait des secondes à se vider).
      if (iTerm > TH_I_MAX)       { iTerm =  TH_I_MAX; thetaInt = iTerm / cfg.kThi; }
      else if (iTerm < -TH_I_MAX) { iTerm = -TH_I_MAX; thetaInt = iTerm / cfg.kThi; }
    } else {
      thetaInt = 0.0f;
    }

    // Retour d'état linéaire autour de la verticale (PD étendu / LQR simplifié)
    return -(cfg.kAlpha * s.alpha
           + cfg.kAdot  * s.alphaDot
           + cfg.kTh    * s.theta
           + cfg.kThd   * s.thetaDot
           + iTerm);
  }

  thetaInt = 0.0f;   // phase SWINGUP : pas d'intégrale

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
