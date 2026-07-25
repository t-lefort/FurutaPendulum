#pragma once
#include "config.h"

namespace ControlClassic {
  enum Phase : uint8_t { SWINGUP, BALANCE };

  void  reset(bool balanceOnly);
  // Retourne la commande duty [-1, 1]. À appeler à 1 kHz.
  float update(const PendulumState &s);
  Phase phase();
}
