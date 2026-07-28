#pragma once
#include "config.h"
#include "qlearning.h"

namespace Storage {
  bool begin();
  bool available();

  // Poids SPL1 : format strictement identique a SplitTileQLearning.save_bin().
  bool saveQTable(const char *path, const float *data, size_t count);
  bool loadQTable(const char *path, float *data, size_t count);
  uint32_t qChecksum(const float *data, size_t count);

  // Etat de progression separe : un fichier SPL1 de simulation reste portable.
  bool saveQLState(const char *path, const QLearning::PersistState &state,
                   uint32_t weightsChecksum);
  bool loadQLState(const char *path, QLearning::PersistState &state,
                   uint32_t weightsChecksum);

  // Logging CSV (depuis loop(), jamais depuis l'ISR).
  bool logStart();
  void logRow(uint32_t ms, uint8_t mode, uint32_t episode,
              const PendulumState &s, int8_t action,
              float stepReward, float episodeReward, float epsilon,
              float epsilonTop, float learningRate);
  void logStop();
}
