#pragma once
#include "config.h"

namespace QLearning {
  // Geometrie exacte du format SPL1 partage avec sim/tiles.py.
  constexpr uint32_t GLOBAL_PER_TILING =
      (uint32_t)TC_GLOBAL_N_ALPHA *
      (uint32_t)(TC_GLOBAL_N_ADOT + 1) *
      (uint32_t)(TC_GLOBAL_N_TDOT + 1);
  constexpr uint32_t LOCAL_PER_TILING =
      (uint32_t)(TC_LOCAL_N_ALPHA + 1) *
      (uint32_t)(TC_LOCAL_N_ADOT + 1) *
      (uint32_t)(TC_LOCAL_N_TDOT + 1);
  constexpr uint32_t GLOBAL_FEATURES =
      (uint32_t)TC_GLOBAL_TILINGS * GLOBAL_PER_TILING;
  constexpr uint32_t LOCAL_FEATURES =
      (uint32_t)TC_LOCAL_TILINGS * LOCAL_PER_TILING;
  constexpr uint32_t FEATURE_COUNT = GLOBAL_FEATURES + LOCAL_FEATURES;
  constexpr size_t WEIGHT_COUNT = (size_t)FEATURE_COUNT * QL_N_ACT;
  constexpr size_t WEIGHT_BYTES = WEIGHT_COUNT * sizeof(float);

  struct Stats {
    uint32_t episode;
    uint32_t topExploreSteps;
    float    epsilon;
    float    epsilonTop;
    float    learningRate;
    float    episodeReward;
    float    bestReward;
    int8_t   lastAction;       // -3..+3
    float    uCommand;         // couple normalise [-1,1]
    float    lastStepReward;
  };

  // Petit etat d'entrainement sauve a cote des poids. Les poids eux-memes
  // restent au format SPL1, directement compatible avec le simulateur.
  struct PersistState {
    uint32_t episode;
    uint32_t topExploreSteps;
    float epsilon;
    float epsilonTop;
    float learningRate;
    float bestReward;
  };

  void begin();
  void startSession(bool greedy);       // greedy=true : aucune mise a jour
  float step(const PendulumState &s, bool &newEpisode);
  void endEpisode();
  bool isPaused();

  const Stats& stats();

  float* table();                       // poids SPL1 bruts
  size_t tableCount();
  void resetTable();
  void resetTrainingState();
  void getPersistState(PersistState &out);
  bool restoreTrainingState(const PersistState &in);
}
