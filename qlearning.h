#pragma once
#include "config.h"

namespace QLearning {
  struct Stats {
    uint32_t episode;
    float    epsilon;
    float    episodeReward;
    float    bestReward;
    int8_t   lastAction;     // -3..+3
    float    uCommand;       // couple normalisé [-1,1] appliqué au moteur
    float    lastStepReward;
  };

  void begin();                       // init table à zéro
  void startSession(bool greedy);     // greedy=true : exploitation pure, pas d'update
  // Un pas RL (à appeler à 50 Hz). Retourne le COUPLE NORMALISÉ [-1,1] à
  // appliquer directement au moteur (aucune boucle de vitesse intermédiaire).
  // newEpisode est mis à true quand un épisode vient de se terminer.
  float step(const PendulumState &s, bool &newEpisode);
  void  endEpisode();                 // fin anticipée (stop utilisateur / faute)

  // true pendant la pause entre deux épisodes. L'agent est inhibé (aucune
  // action, aucune mise à jour de Q) et l'appelant doit COUPER le moteur :
  // on attend simplement que pendule et bras s'immobilisent. Aucun couple
  // n'est piloté, donc aucun emballement possible. Au sortir de la pause,
  // theta est re-zéroté (Encoders::rezeroArm) : chaque épisode part de 0.
  bool isPaused();

  const Stats& stats();

  // Accès brut pour la sauvegarde SD
  float*  table();
  size_t  tableCount();               // nombre de floats
  void    resetTable();
}
