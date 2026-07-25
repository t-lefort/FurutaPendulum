#pragma once
#include "config.h"

namespace Safety {
  void reset();
  // Retourne FAULT_NONE si tout va bien, sinon le code de défaut.
  // À appeler à 1 kHz, uniquement quand le moteur est actif.
  FaultCode check(const PendulumState &s, bool motorSaturated);
}
