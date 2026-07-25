#include "qlearning.h"

// Table dans la RAM2 (DMAMEM) : 49*31*7 floats = ~42 kB, laisse la RAM1 au code
DMAMEM static float Q[QL_N_ALPHA * QL_N_ADOT * QL_N_ACT];

static QLearning::Stats st;
static bool  greedyMode = false;
static int   prevStateIdx = -1;
static int   prevAction   = 0;
static float stepsInEpisode = 0;
// Sequence de remise en place entre deux episodes : on ramene le bras vers
// theta = 0, puis moteur coupe on attend que le pendule pende immobile, pour
// que chaque episode reparte du meme etat.
static QLearning::ResetPhase rsPhase = QLearning::RS_NONE;
static float resetTime = 0.0f;
// Actions = couples normalises appliques DIRECTEMENT au moteur.
static const float ACTION_U[QL_N_ACT] = {
  -QL_U_MAX, -QL_U_MAX * 0.66f, -QL_U_MAX * 0.33f, 0.0f,
   QL_U_MAX * 0.33f,  QL_U_MAX * 0.66f,  QL_U_MAX };

static inline int binAlpha(float a) {
  // alpha dans [-pi, pi] -> [0, N-1]
  int b = (int)((a + PI) / TWO_PI * QL_N_ALPHA);
  return constrain(b, 0, QL_N_ALPHA - 1);
}
static inline int binAdot(float w) {
  w = constrain(w, -QL_ADOT_MAX, QL_ADOT_MAX);
  int b = (int)((w + QL_ADOT_MAX) / (2.0f * QL_ADOT_MAX) * QL_N_ADOT);
  return constrain(b, 0, QL_N_ADOT - 1);
}
static inline int stateIndex(const PendulumState &s) {
  return (binAlpha(s.alpha) * QL_N_ADOT + binAdot(s.alphaDot)) * QL_N_ACT;
}
// En cas d'EGALITE (typiquement une table vierge, tout a zero), on doit
// retomber sur l'action NEUTRE (couple nul) et non sur l'indice 0, qui vaut
// -QL_U_MAX : sinon un agent non entraine applique le couple maxi dans un
// sens en permanence et le bras part en toupie jusqu'a la faute "plage bras".
static constexpr int ACT_NEUTRAL = QL_N_ACT / 2;   // ACTION_U[3] = couple nul

static inline int bestAction(int sIdx) {
  int best = ACT_NEUTRAL; float bv = Q[sIdx + ACT_NEUTRAL];
  for (int a = 0; a < QL_N_ACT; a++)
    if (Q[sIdx + a] > bv) { bv = Q[sIdx + a]; best = a; }
  return best;
}
static inline float maxQ(int sIdx) {
  float bv = Q[sIdx];
  for (int a = 1; a < QL_N_ACT; a++) if (Q[sIdx + a] > bv) bv = Q[sIdx + a];
  return bv;
}

static float reward(const PendulumState &s, int action) {
  float r = 2.0f * cosf(s.alpha)
          - 0.02f  * fabsf(s.alphaDot)
          - 0.05f  * fabsf(s.thetaDot)     // etait 0.005 : trop faible pour
                                           // decourager la rotation continue
          - 0.02f  * fabsf(ACTION_U[action]) / QL_U_MAX;
  // Les bonus "pendule en haut" exigent desormais un bras LENT. Sans cette
  // condition, un bras qui tourne a fond maintient le pendule releve par
  // effet centrifuge et touche les bonus sans jamais equilibrer : c'est un
  // optimum local tres attractif dont l'agent ne ressort plus.
  if (fabsf(s.alpha) < radians(10) && fabsf(s.thetaDot) < 3.0f) r += 5.0f;
  if (fabsf(s.alpha) < radians(5) && fabsf(s.alphaDot) < 1.0f
                                  && fabsf(s.thetaDot) < 2.0f) r += 20.0f;
  return r;
}

void QLearning::begin() {
  resetTable();
  st = {};
  st.epsilon = QL_EPS0;
  st.bestReward = -1e9f;
}

void QLearning::resetTable() {
  for (size_t i = 0; i < tableCount(); i++) Q[i] = 0.0f;
}

void QLearning::startSession(bool greedy) {
  greedyMode = greedy;
  prevStateIdx = -1;
  stepsInEpisode = 0;
  st.episodeReward = 0.0f;
  rsPhase   = QLearning::RS_NONE;
  resetTime = 0.0f;
}

static inline void beginReset() {
  rsPhase   = QLearning::RS_RETURN;
  resetTime = 0.0f;
}

// Fin de la sequence : l'episode suivant peut demarrer.
static inline void endReset() {
  rsPhase          = QLearning::RS_NONE;
  resetTime        = 0.0f;
  prevStateIdx     = -1;    // pas de transition a cheval sur le reset
  stepsInEpisode   = 0;
  st.episodeReward = 0.0f;
}

float QLearning::step(const PendulumState &s, bool &newEpisode) {
  newEpisode = false;

  // ---- Remise en place entre deux episodes (agent inhibe) ----
  // 1) RS_RETURN : ramener le bras vers theta = 0 (sinon l'episode suivant
  //    repartirait bras deja en butee et se terminerait aussitot).
  // 2) RS_SETTLE : moteur coupe, attendre que le pendule pende immobile, pour
  //    que tous les episodes partent du meme etat initial.
  if (rsPhase != QLearning::RS_NONE) {
    resetTime += RL_DT;
    st.uCommand   = 0.0f;    // l'agent est inhibe pendant toute la sequence
    st.lastAction = 0;

    if (rsPhase == QLearning::RS_RETURN) {
      const bool armHome = fabsf(s.theta)    < QL_RESET_TOL_RAD &&
                           fabsf(s.thetaDot) < 1.0f;
      // Sur expiration du delai on accepte un retour PARTIEL, mais jamais tant
      // que le bras est encore hors plage : relancer dans cet etat le ferait
      // repartir de plus loin a chaque episode (effet cliquet) jusqu'a la
      // faute "plage bras". Mieux vaut continuer a ramener.
      const bool inRange = (QL_THETA_TURNS <= 0.0f ||
                            fabsf(s.theta) < QL_THETA_TURNS * (float)TWO_PI);
      if (armHome || (resetTime > QL_RESET_MAX_S && inRange)) {
        rsPhase   = QLearning::RS_SETTLE;   // -> moteur coupe, on laisse pendre
        resetTime = 0.0f;
      }
      return 0.0f;
    }

    // ---- RS_SETTLE : moteur coupe (par l'appelant), on attend le repos ----
    if (fabsf(s.theta) > QL_SETTLE_DRIFT_RAD) {   // bras parti a la derive
      rsPhase   = QLearning::RS_RETURN;
      resetTime = 0.0f;
      return 0.0f;
    }
    const bool hanging = fabsf(s.alpha)    > (float)PI - QL_SETTLE_RAD &&
                         fabsf(s.alphaDot) < QL_SETTLE_ADOT;
    if (!hanging && resetTime <= QL_SETTLE_MAX_S) return 0.0f;
    endReset();     // pret : on enchaine sur le nouvel episode ci-dessous
  }

  const int sIdx = stateIndex(s);
  // Etat TERMINAL de l'episode (et non coupure du mode) : bras hors plage, ou
  // bras emballe. Couper tot sur la vitesse limite l'elan a freiner ensuite.
  const bool outOfRange =
      (QL_THETA_TURNS > 0.0f &&
       fabsf(s.theta) > QL_THETA_TURNS * (float)TWO_PI) ||
      (QL_TDOT_MAX > 0.0f && fabsf(s.thetaDot) > QL_TDOT_MAX);

  // Mise à jour Q(s,a) avec la transition précédente
  if (!greedyMode && prevStateIdx >= 0) {
    float r = reward(s, prevAction);
    if (outOfRange) r += QL_R_OUT_RANGE;
    st.lastStepReward = r;
    st.episodeReward += r;
    float &q = Q[prevStateIdx + prevAction];
    // Sur un etat terminal on ne bootstrappe PAS sur l'etat suivant : la
    // penalite doit rester attachee a l'action qui y a mene.
    const float target = outOfRange ? r : (r + QL_GAMMA * maxQ(sIdx));
    q += QL_LR * (target - q);
  }

  if (outOfRange) {
    // Un episode "vide" (sortie de plage des le premier pas, typiquement si le
    // retour precedent a echoue) ne doit PAS compter : sinon epsilon decroit
    // sans qu'aucun apprentissage n'ait eu lieu.
    if (stepsInEpisode > 0.0f) { endEpisode(); newEpisode = true; }
    else                       { prevStateIdx = -1; }
    beginReset();
    st.uCommand = 0.0f;
    return 0.0f;
  }

  // Choix de l'action (epsilon-greedy)
  int a;
  if (!greedyMode && (random(10000) / 10000.0f) < st.epsilon)
    a = random(QL_N_ACT);
  else
    a = bestAction(sIdx);

  prevStateIdx = sIdx;
  prevAction   = a;
  st.lastAction = (int8_t)(a - ACT_NEUTRAL);
  st.uCommand   = ACTION_U[a];

  // Gestion de l'épisode
  stepsInEpisode += 1.0f;
  if (stepsInEpisode * RL_DT >= QL_EPISODE_S) {
    endEpisode();
    newEpisode = true;
    beginReset();       // ramene le bras avant l'episode suivant
  }
  return ACTION_U[a];
}

void QLearning::endEpisode() {
  if (!greedyMode) {
    st.episode++;
    if (st.episodeReward > st.bestReward) st.bestReward = st.episodeReward;
    st.epsilon = max(QL_EPS_MIN, st.epsilon * QL_EPS_DECAY);
  }
  st.episodeReward = 0.0f;
  stepsInEpisode = 0;
  prevStateIdx = -1;
}

QLearning::ResetPhase QLearning::resetPhase() { return rsPhase; }

const QLearning::Stats& QLearning::stats() { return st; }
float*  QLearning::table()      { return Q; }
size_t  QLearning::tableCount() { return (size_t)QL_N_ALPHA * QL_N_ADOT * QL_N_ACT; }
