#pragma once
#include "config.h"

namespace QLearning {
  struct Stats {
    uint32_t episode;
    float    epsilon;
    float    episodeReward;
    float    bestReward;
    int8_t   lastAction;     // -3..+3
    float    wCommand;       // rad/s, consigne vitesse bras courante
    float    lastStepReward;
  };

  void begin();                       // init table à zéro
  void startSession(bool greedy);     // greedy=true : exploitation pure, pas d'update
  // Un pas RL (à appeler à 50 Hz). Retourne la consigne de vitesse bras (rad/s).
  // newEpisode est mis à true quand un épisode vient de se terminer.
  float step(const PendulumState &s, bool &newEpisode);
  void  endEpisode();                 // fin anticipée (stop utilisateur / faute)
  // true entre deux épisodes, pendant le retour du bras vers theta = 0.
  // L'appelant doit alors piloter le moteur en COUPLE (PD sur theta) au lieu
  // d'utiliser la consigne de vitesse : voir QL_RESET_* dans config.h.
  bool  isResetting();

  const Stats& stats();

  // Accès brut pour la sauvegarde SD
  float*  table();
  size_t  tableCount();               // nombre de floats
  void    resetTable();
}
