#include "qlearning.h"
#include "encoders.h"   // rezeroArm() au debut de chaque episode

// Table dans la RAM2 (DMAMEM) : 49*31*7 floats = ~42 kB, laisse la RAM1 au code
DMAMEM static float Q[QL_N_ALPHA * QL_N_ADOT * QL_N_ACT];

static QLearning::Stats st;
static bool  greedyMode = false;
static int   prevStateIdx = -1;
static int   prevAction   = 0;
static float stepsInEpisode = 0;
// Pause entre deux episodes : moteur coupe, on attend que tout s'immobilise
// pour que chaque episode reparte du meme etat (pendule en bas, au repos).
static bool  paused    = false;
static float pauseTime = 0.0f;
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
  // On demarre PAR la pause : moteur coupe tant que le pendule n'est pas
  // retombe immobile (l'utilisateur peut lancer le mode pendule en mouvement).
  paused    = true;
  pauseTime = 0.0f;
}

static inline void beginPause() { paused = true; pauseTime = 0.0f; }

float QLearning::step(const PendulumState &s, bool &newEpisode) {
  newEpisode = false;

  // ---- Pause entre deux episodes (agent inhibe, moteur coupe par l'appelant) ----
  // Pas de retour actif du bras : theta n'est pas observe par l'agent et le
  // collecteur tournant autorise n'importe quelle position. On attend juste
  // que tout s'immobilise, puis on RE-ZERO theta (offset logiciel) : chaque
  // episode repart ainsi de theta = 0 sans qu'aucun couple ne soit pilote,
  // et la derive ne peut pas s'accumuler vers TurnsMax d'episode en episode.
  if (paused) {
    pauseTime += RL_DT;
    st.uCommand   = 0.0f;
    st.lastAction = 0;
    const bool settled = fabsf(s.alpha)    > (float)PI - QL_SETTLE_RAD &&
                         fabsf(s.alphaDot) < QL_SETTLE_ADOT &&
                         fabsf(s.thetaDot) < QL_SETTLE_TDOT;
    if (!settled && pauseTime <= QL_SETTLE_MAX_S) return 0.0f;
    // Pret (ou delai ecoule : on accepte l'etat quasi-stabilise).
    Encoders::rezeroArm();     // theta := 0, compteurs materiels intacts (FOC)
    paused           = false;
    pauseTime        = 0.0f;
    prevStateIdx     = -1;     // pas de transition a cheval sur la pause
    stepsInEpisode   = 0;
    st.episodeReward = 0.0f;
    // NB : s.theta date d'avant le re-zero ; il n'est pas utilise ci-dessous
    // au premier pas (l'etat RL est [alpha, alphaDot] et la sortie de plage
    // n'est evaluee qu'a partir du 2e pas).
  }

  const int sIdx = stateIndex(s);
  // Etat TERMINAL de l'episode (et non coupure du mode) : bras trop loin de
  // son point de depart, ou bras emballe. Jamais au premier pas (theta vient
  // d'etre re-zeroe, la valeur recue ici peut etre anterieure au re-zero).
  const bool outOfRange = stepsInEpisode > 0.0f &&
      ((QL_THETA_TURNS > 0.0f &&
        fabsf(s.theta) > QL_THETA_TURNS * (float)TWO_PI) ||
       (QL_TDOT_MAX > 0.0f && fabsf(s.thetaDot) > QL_TDOT_MAX));

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
    endEpisode();
    newEpisode = true;
    beginPause();
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
    beginPause();       // moteur coupe, on laisse tout retomber
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

bool QLearning::isPaused() { return paused; }

const QLearning::Stats& QLearning::stats() { return st; }
float*  QLearning::table()      { return Q; }
size_t  QLearning::tableCount() { return (size_t)QL_N_ALPHA * QL_N_ADOT * QL_N_ACT; }
