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

  // Séquence entre deux épisodes. L'agent est inhibé pendant toute la
  // séquence (aucune action choisie, aucune mise à jour de Q) ; c'est
  // l'appelant qui pilote le moteur selon la phase :
  //   RS_RETURN — ramener le bras vers theta = 0 (PD en couple, QL_RESET_*)
  //   RS_SETTLE — MOTEUR COUPÉ, on attend que le pendule pende immobile
  enum ResetPhase : uint8_t { RS_NONE = 0, RS_RETURN, RS_SETTLE };
  ResetPhase resetPhase();

  const Stats& stats();

  // Accès brut pour la sauvegarde SD
  float*  table();
  size_t  tableCount();               // nombre de floats
  void    resetTable();
}
