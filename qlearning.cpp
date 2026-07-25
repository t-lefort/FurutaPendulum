#include "qlearning.h"

// Table dans la RAM2 (DMAMEM) : 49*31*7 floats = ~42 kB, laisse la RAM1 au code
DMAMEM static float Q[QL_N_ALPHA * QL_N_ADOT * QL_N_ACT];

static QLearning::Stats st;
static bool  greedyMode = false;
static int   prevStateIdx = -1;
static int   prevAction   = 0;
static float stepsInEpisode = 0;
static const float ACTION_W[QL_N_ACT] = {
  -QL_W_MAX, -QL_W_MAX * 0.66f, -QL_W_MAX * 0.33f, 0.0f,
   QL_W_MAX * 0.33f,  QL_W_MAX * 0.66f,  QL_W_MAX };

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
static inline int bestAction(int sIdx) {
  int best = 0; float bv = Q[sIdx];
  for (int a = 1; a < QL_N_ACT; a++)
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
          - 0.005f * fabsf(s.thetaDot)
          - 0.02f  * fabsf(ACTION_W[action]) / QL_W_MAX;
  if (fabsf(s.alpha) < radians(10)) r += 5.0f;
  if (fabsf(s.alpha) < radians(5) && fabsf(s.alphaDot) < 1.0f) r += 20.0f;
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
}

float QLearning::step(const PendulumState &s, bool &newEpisode) {
  newEpisode = false;
  const int sIdx = stateIndex(s);

  // Mise à jour Q(s,a) avec la transition précédente
  if (!greedyMode && prevStateIdx >= 0) {
    const float r = reward(s, prevAction);
    st.lastStepReward = r;
    st.episodeReward += r;
    float &q = Q[prevStateIdx + prevAction];
    q += QL_LR * (r + QL_GAMMA * maxQ(sIdx) - q);
  }

  // Choix de l'action (epsilon-greedy)
  int a;
  if (!greedyMode && (random(10000) / 10000.0f) < st.epsilon)
    a = random(QL_N_ACT);
  else
    a = bestAction(sIdx);

  prevStateIdx = sIdx;
  prevAction   = a;
  st.lastAction = (int8_t)(a - 3);
  st.wCommand   = ACTION_W[a];

  // Gestion de l'épisode
  stepsInEpisode += 1.0f;
  if (stepsInEpisode * RL_DT >= QL_EPISODE_S) {
    endEpisode();
    newEpisode = true;
  }
  return ACTION_W[a];
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

const QLearning::Stats& QLearning::stats() { return st; }
float*  QLearning::table()      { return Q; }
size_t  QLearning::tableCount() { return (size_t)QL_N_ALPHA * QL_N_ADOT * QL_N_ACT; }
