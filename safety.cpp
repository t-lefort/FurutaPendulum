#include "safety.h"
#include "settings.h"
using Settings::cfg;

static float satTime = 0.0f;

void Safety::reset() { satTime = 0.0f; }

FaultCode Safety::check(const PendulumState &s, bool motorSaturated) {
  if (fabsf(s.alphaDot) > cfg.alphaDotMax) return FAULT_ALPHA_DOT;
  if (fabsf(s.thetaDot) > cfg.thetaDotMax) return FAULT_THETA_DOT;

  if (cfg.thetaTurnsMax > 0.0f &&
      fabsf(s.theta) > cfg.thetaTurnsMax * TWO_PI) return FAULT_THETA_RANGE;

  if (motorSaturated) {
    satTime += CTRL_DT;
    if (satTime > SAT_TIMEOUT_S) return FAULT_SATURATION;
  } else {
    satTime = 0.0f;
  }
  return FAULT_NONE;
}
