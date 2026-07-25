#pragma once
#include "config.h"

namespace Storage {
  bool begin();                 // init microSD (retourne false si absente)
  bool available();

  bool saveQTable(const char *path, const float *data, size_t count);
  bool loadQTable(const char *path, float *data, size_t count);

  // Logging CSV (à appeler depuis loop(), jamais depuis la boucle 1 kHz)
  bool logStart();
  void logRow(uint32_t ms, uint8_t mode, uint32_t episode,
              const PendulumState &s, int8_t action,
              float stepReward, float episodeReward, float epsilon);
  void logStop();
}
