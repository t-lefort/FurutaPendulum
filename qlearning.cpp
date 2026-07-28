#include "qlearning.h"
#include "encoders.h"
#include "settings.h"
#include <math.h>
#include <string.h>

using Settings::cfg;

namespace {
constexpr int ACT_NEUTRAL = QL_N_ACT / 2;
constexpr int QL_N_LVL = ACT_NEUTRAL;
constexpr uint8_t MAX_ACTIVE =
    (TC_GLOBAL_TILINGS > TC_LOCAL_TILINGS)
        ? TC_GLOBAL_TILINGS : TC_LOCAL_TILINGS;

// Les 455,4 Kio de poids occupent la RAM2. Les 4 Kio de traces, tres souvent
// parcourus dans l'ISR, restent en RAM1 rapide ou la marge est confortable.
DMAMEM static float weights[QLearning::WEIGHT_COUNT];

struct Trace {
  uint32_t key;   // feature * QL_N_ACT + action
  float value;
};
static Trace traces[QL_TRACE_MAX];

static_assert(TC_SPLIT == 1, "Le firmware utilise la representation split SPL1");
static_assert(TC_SPLIT_OVERLAP == 0,
              "Le profil valide utilise un gate dur sans chevauchement");
static_assert(QL_N_ACT == 7, "Le format de politique valide utilise 7 actions");
static_assert(MAX_ACTIVE <= 16, "Augmenter la taille des tableaux de traits");
static_assert(QLearning::WEIGHT_BYTES <= 512u * 1024u,
              "Les poids depassent les 512 Kio de RAM2");

QLearning::Stats st;
bool greedyMode = false;
bool paused = false;
float pauseTime = 0.0f;
float stepsInEpisode = 0.0f;
float episodeLimitS = QL_EPISODE_S;

uint32_t prevFeatures[MAX_ACTIVE];
uint8_t prevFeatureCount = 0;
int prevAction = ACT_NEUTRAL;
float prevPhi = 0.0f;

uint16_t traceCount = 0;
int exploreHold = 0;
int exploreAct = ACT_NEUTRAL;
float kickTime = 0.0f;
bool firstUpSeen = false;
bool afterUpArmed = false;
float upHoldTime = 0.0f;

static inline float actionU(int action) {
  const int k = action - ACT_NEUTRAL;
  if (k == 0) return 0.0f;
  const float uMin = cfg.qlUMin;
  const float uMax = fmaxf(cfg.qlUMax, uMin);
  const float t = (QL_N_LVL > 1)
      ? (float)(abs(k) - 1) / (float)(QL_N_LVL - 1) : 1.0f;
  const float u = uMin + t * (uMax - uMin);
  return (k > 0) ? u : -u;
}

static inline int tileCoordinate(float x, float lo, float hi, int n,
                                 bool wrap, int dimension, int tiling,
                                 int tilingCount) {
  const float width = (hi - lo) / (float)n;
  const float offset = width *
      (float)(((2 * dimension + 1) * tiling) % tilingCount) /
      (float)tilingCount;
  int c = (int)floorf((x + offset - lo) / width);
  if (wrap) {
    c %= n;
    if (c < 0) c += n;
  } else {
    if (c < 0) c = 0;
    if (c > n) c = n;
  }
  return c;
}

static uint8_t activeFeatures(const PendulumState &s, uint32_t *out) {
  if (fabsf(s.alpha) < TC_LOCAL_RAD) {
    for (int t = 0; t < TC_LOCAL_TILINGS; ++t) {
      const int ca = tileCoordinate(
          s.alpha, -TC_LOCAL_RAD, TC_LOCAL_RAD, TC_LOCAL_N_ALPHA,
          false, 0, t, TC_LOCAL_TILINGS);
      const int cw = tileCoordinate(
          s.alphaDot, -TC_LOCAL_ADOT_MAX, TC_LOCAL_ADOT_MAX, TC_LOCAL_N_ADOT,
          false, 1, t, TC_LOCAL_TILINGS);
      const int ct = tileCoordinate(
          s.thetaDot, -TC_LOCAL_TDOT_MAX, TC_LOCAL_TDOT_MAX, TC_LOCAL_N_TDOT,
          false, 2, t, TC_LOCAL_TILINGS);
      const uint32_t idx =
          ((uint32_t)ca * (TC_LOCAL_N_ADOT + 1) + (uint32_t)cw) *
          (TC_LOCAL_N_TDOT + 1) + (uint32_t)ct;
      out[t] = QLearning::GLOBAL_FEATURES +
               (uint32_t)t * QLearning::LOCAL_PER_TILING + idx;
    }
    return TC_LOCAL_TILINGS;
  }

  for (int t = 0; t < TC_GLOBAL_TILINGS; ++t) {
    const int ca = tileCoordinate(
        s.alpha, -(float)PI, (float)PI, TC_GLOBAL_N_ALPHA,
        true, 0, t, TC_GLOBAL_TILINGS);
    const int cw = tileCoordinate(
        s.alphaDot, -QL_ADOT_MAX, QL_ADOT_MAX, TC_GLOBAL_N_ADOT,
        false, 1, t, TC_GLOBAL_TILINGS);
    const int ct = tileCoordinate(
        s.thetaDot, -QL_TDOT_BIN_MAX, QL_TDOT_BIN_MAX, TC_GLOBAL_N_TDOT,
        false, 2, t, TC_GLOBAL_TILINGS);
    const uint32_t idx =
        ((uint32_t)ca * (TC_GLOBAL_N_ADOT + 1) + (uint32_t)cw) *
        (TC_GLOBAL_N_TDOT + 1) + (uint32_t)ct;
    out[t] = (uint32_t)t * QLearning::GLOBAL_PER_TILING + idx;
  }
  return TC_GLOBAL_TILINGS;
}

static inline float qValue(const uint32_t *features, uint8_t count, int action) {
  float q = 0.0f;
  for (uint8_t i = 0; i < count; ++i)
    q += weights[features[i] * QL_N_ACT + action];
  return q;
}

static int bestAction(const uint32_t *features, uint8_t count) {
  int best = ACT_NEUTRAL;
  float bestValue = qValue(features, count, best);
  for (int action = 0; action < QL_N_ACT; ++action) {
    if (action == ACT_NEUTRAL) continue;
    const float value = qValue(features, count, action);
    if (value > bestValue) {
      bestValue = value;
      best = action;
    }
  }
  return best;
}

static inline float featureRate(uint32_t feature) {
  if (feature >= QLearning::GLOBAL_FEATURES)
    return st.learningRate * TC_LOCAL_LR_SCALE / (float)TC_LOCAL_TILINGS;
  return st.learningRate * TC_GLOBAL_LR_SCALE / (float)TC_GLOBAL_TILINGS;
}

static void removeFeatureTraces(uint32_t feature) {
  uint16_t dst = 0;
  for (uint16_t src = 0; src < traceCount; ++src) {
    if (traces[src].key / QL_N_ACT != feature)
      traces[dst++] = traces[src];
  }
  traceCount = dst;
}

static void updateWeights(const uint32_t *features, uint8_t count, int action,
                          float delta, bool terminal) {
  if (QL_LAMBDA <= 0.0f) {
    for (uint8_t i = 0; i < count; ++i) {
      const uint32_t key = features[i] * QL_N_ACT + action;
      weights[key] += featureRate(features[i]) * delta;
    }
    return;
  }

  // Traces remplacantes : une seule action reste eligible par trait actif.
  for (uint8_t i = 0; i < count; ++i)
    removeFeatureTraces(features[i]);

  const uint16_t overflow =
      (traceCount + count > QL_TRACE_MAX)
          ? (uint16_t)(traceCount + count - QL_TRACE_MAX) : 0;
  if (overflow > 0) {
    memmove(traces, traces + overflow,
            (traceCount - overflow) * sizeof(Trace));
    traceCount -= overflow;
  }
  for (uint8_t i = 0; i < count; ++i) {
    traces[traceCount++] = {
        features[i] * QL_N_ACT + (uint32_t)action, 1.0f};
  }

  const float decay = QL_GAMMA * QL_LAMBDA;
  uint16_t dst = 0;
  for (uint16_t src = 0; src < traceCount; ++src) {
    Trace tr = traces[src];
    const uint32_t feature = tr.key / QL_N_ACT;
    weights[tr.key] += featureRate(feature) * delta * tr.value;
    tr.value *= decay;
    if (tr.value >= QL_TRACE_MIN)
      traces[dst++] = tr;
  }
  traceCount = terminal ? 0 : dst;
}

static float stateReward(const PendulumState &s, int action) {
  float r = 1.0f + cosf(s.alpha);
  if (fabsf(s.alpha) < QL_ADOT_TOP_RAD)
    r -= QL_K_ADOT_TOP * fabsf(s.alphaDot);
  if (fabsf(s.alpha) < QL_TDOT_TOP_RAD)
    r -= QL_K_TDOT_TOP * s.thetaDot * s.thetaDot;

  if (QL_K_BAL > 0.0f) {
    const float ar = s.alpha / QL_BAL_CONE_RAD;
    const float wr = s.alphaDot / fmaxf(QL_BAL_CONE_ADOT, 1e-6f);
    const float cone = fmaxf(0.0f, 1.0f - ar * ar - wr * wr);
    const float taper = (QL_BAL_CONE_TDOT > 0.0f)
        ? fmaxf(0.0f, 1.0f -
            (s.thetaDot / QL_BAL_CONE_TDOT) *
            (s.thetaDot / QL_BAL_CONE_TDOT))
        : 1.0f;
    r += QL_K_BAL * cone * taper;
  }

  const float tdOver = fabsf(s.thetaDot) - QL_TDOT_SOFT;
  if (tdOver > 0.0f) r -= QL_K_TDOT * tdOver;
  r -= 0.02f * fabsf(actionU(action)) / fmaxf(cfg.qlUMax, 1e-3f);
  if (fabsf(s.alpha) < QL_UP_RAD && fabsf(s.thetaDot) < QL_UP_TDOT)
    r += QL_R_UP;
  if (fabsf(s.alpha) < QL_BAL_RAD &&
      fabsf(s.alphaDot) < QL_BAL_ADOT &&
      fabsf(s.thetaDot) < QL_BAL_TDOT)
    r += QL_R_BAL;
  return r;
}

static float potential(const PendulumState &s) {
  float phi = 0.0f;
  if (QL_K_ENERGY > 0.0f) {
    // Le RL reste aligne sur le modele valide en simulation, sans ecraser les
    // anciens parametres EEPROM dont depend le controle classique.
    const float pendJ = PEND_J_ROD_BOB
        ? PEND_MASS * PEND_LEN * (4.0f * PEND_LCOM - PEND_LEN) / 3.0f
        : PEND_MASS * PEND_LEN * PEND_LEN / 3.0f;
    const float eTop = fmaxf(PEND_MASS * G_GRAV * PEND_LCOM, 1e-6f);
    const float e = 0.5f * pendJ * s.alphaDot * s.alphaDot +
                    eTop * cosf(s.alpha);
    phi -= QL_K_ENERGY * fabsf(e - eTop) / (2.0f * eTop);
  }
  if (QL_K_APPROACH > 0.0f) {
    const float height = 0.5f * (1.0f + cosf(s.alpha));
    const float wa = s.alphaDot / fmaxf(QL_APPROACH_ADOT, 1e-6f);
    const float wt = s.thetaDot / fmaxf(QL_APPROACH_TDOT, 1e-6f);
    phi += QL_K_APPROACH * height / (1.0f + wa * wa + wt * wt);
  }
  return phi;
}

static void noteFirstUp(float alpha) {
  if (fabsf(alpha) >= QL_FIRST_UP_RAD) {
    upHoldTime = 0.0f;
    return;
  }
  upHoldTime += RL_DT;
  if (upHoldTime >= QL_AFTER_UP_ARM_S)
    afterUpArmed = true;
  if (!firstUpSeen && stepsInEpisode > 0.0f) {
    firstUpSeen = true;
    episodeLimitS += QL_FIRST_UP_BONUS_S;
  }
}

static bool terminalCheck(const PendulumState &s, float &penalty) {
  penalty = 0.0f;
  if (stepsInEpisode <= 0.0f) return false;
  if (QL_AFTER_UP_FALL_RAD > 0.0f && afterUpArmed &&
      fabsf(s.alpha) > QL_AFTER_UP_FALL_RAD)
    return true;
  if ((QL_THETA_TURNS > 0.0f &&
       fabsf(s.theta) > QL_THETA_TURNS * (float)TWO_PI) ||
      (QL_TDOT_MAX > 0.0f && fabsf(s.thetaDot) > QL_TDOT_MAX)) {
    penalty = QL_R_OUT_RANGE;
    return true;
  }
  return false;
}

static int kickAction(float alpha, float alphaDot) {
  if (QL_KICK <= 0.0f) return -1;
  if (fabsf(alphaDot) < SWING_KICK_ADOT &&
      fabsf(alpha) > (float)PI - SWING_KICK_RAD) {
    kickTime += RL_DT;
    const float period = 2.0f * SWING_KICK_HALF_S;
    return fmodf(kickTime, period) < SWING_KICK_HALF_S
        ? QL_N_ACT - 1 : 0;
  }
  kickTime = 0.0f;
  return -1;
}

static int exploreHoldFor(float alpha) {
  if (QL_EXPLORE_NEAR_RAD > 0.0f &&
      fabsf(alpha) < QL_EXPLORE_NEAR_RAD)
    return max(1, QL_EXPLORE_HOLD_TOP);
  return max(1, QL_EXPLORE_HOLD);
}

static float explorationEpsilon(bool nearTop) {
  if (!nearTop) return st.epsilon;
  if (QL_EPS_TOP0 >= 0.0f) {
    const float eps = st.epsilonTop;
    if (!greedyMode) {
      ++st.topExploreSteps;
      st.epsilonTop =
          fmaxf(QL_EPS_TOP_MIN, st.epsilonTop * QL_EPS_TOP_DECAY);
    }
    return eps;
  }
  return (QL_EXPLORE_EPS_TOP >= 0.0f)
      ? fminf(st.epsilon, QL_EXPLORE_EPS_TOP) : st.epsilon;
}

static int selectAction(const uint32_t *features, uint8_t count,
                        float alpha, float alphaDot) {
  const int kick = kickAction(alpha, alphaDot);
  if (kick >= 0) return kick;
  if (greedyMode) return bestAction(features, count);

  const bool nearTop = QL_EXPLORE_NEAR_RAD > 0.0f &&
                       fabsf(alpha) < QL_EXPLORE_NEAR_RAD;
  if (exploreHold > 0) {
    if (nearTop) {
      exploreHold = 0;
    } else {
      --exploreHold;
      return exploreAct;
    }
  }

  const float eps = explorationEpsilon(nearTop);
  if ((float)random(10000) / 10000.0f < eps) {
    exploreAct = (int)random(QL_N_ACT);
    exploreHold = exploreHoldFor(alpha) - 1;
    return exploreAct;
  }
  return bestAction(features, count);
}

static void beginPause() {
  paused = true;
  pauseTime = 0.0f;
  exploreHold = 0;
}
} // namespace

void QLearning::begin() {
  resetTable();
  resetTrainingState();
}

void QLearning::resetTable() {
  memset(weights, 0, sizeof(weights));
  traceCount = 0;
  prevFeatureCount = 0;
}

void QLearning::resetTrainingState() {
  st = {};
  st.epsilon = QL_EPS0;
  st.epsilonTop = QL_EPS_TOP0;
  st.learningRate = QL_LR;
  st.bestReward = -1e9f;
}

void QLearning::startSession(bool greedy) {
  greedyMode = greedy;
  if (!greedy)
    randomSeed(micros() ^ (st.episode * 0x9E3779B1u));
  prevFeatureCount = 0;
  prevPhi = 0.0f;
  traceCount = 0;
  stepsInEpisode = 0.0f;
  episodeLimitS = QL_EPISODE_S;
  firstUpSeen = false;
  afterUpArmed = false;
  upHoldTime = 0.0f;
  st.episodeReward = 0.0f;
  beginPause();
}

float QLearning::step(const PendulumState &s, bool &newEpisode) {
  newEpisode = false;

  if (paused) {
    pauseTime += RL_DT;
    st.uCommand = 0.0f;
    st.lastAction = 0;
    const bool settled =
        fabsf(s.alpha) > (float)PI - QL_SETTLE_RAD &&
        fabsf(s.alphaDot) < QL_SETTLE_ADOT &&
        fabsf(s.thetaDot) < QL_SETTLE_TDOT;
    if (!settled && pauseTime <= QL_SETTLE_MAX_S)
      return 0.0f;

    Encoders::rezeroArm();
    paused = false;
    pauseTime = 0.0f;
    prevFeatureCount = 0;
    prevPhi = 0.0f;
    traceCount = 0;
    stepsInEpisode = 0.0f;
    st.episodeReward = 0.0f;
  }

  uint32_t features[MAX_ACTIVE];
  const uint8_t featureCount = activeFeatures(s, features);
  noteFirstUp(s.alpha);
  float terminalPenalty = 0.0f;
  const bool terminal = terminalCheck(s, terminalPenalty);
  const float phi = potential(s);

  // SARSA choisit a' avant la mise a jour de (s,a,r,s').
  int nextAction = -1;
  if (QL_SARSA > 0.0f && !terminal)
    nextAction = selectAction(features, featureCount, s.alpha, s.alphaDot);

  if (!greedyMode && prevFeatureCount > 0) {
    float r = stateReward(s, prevAction);
    r += (terminal ? 0.0f : QL_GAMMA * phi) - prevPhi;
    if (terminal) r += terminalPenalty;
    st.lastStepReward = r;
    st.episodeReward += r;

    float target;
    if (terminal) {
      target = r;
    } else if (QL_SARSA > 0.0f) {
      target = r + QL_GAMMA *
          qValue(features, featureCount, nextAction);
    } else {
      const int greedy = bestAction(features, featureCount);
      target = r + QL_GAMMA * qValue(features, featureCount, greedy);
    }
    const float oldQ = qValue(prevFeatures, prevFeatureCount, prevAction);
    updateWeights(prevFeatures, prevFeatureCount, prevAction,
                  target - oldQ, terminal);
  }

  if (terminal) {
    endEpisode();
    beginPause();
    st.uCommand = 0.0f;
    newEpisode = true;
    return 0.0f;
  }

  const int action = (nextAction >= 0)
      ? nextAction
      : selectAction(features, featureCount, s.alpha, s.alphaDot);

  if (QL_LAMBDA > 0.0f && QL_SARSA <= 0.0f && !greedyMode &&
      action != bestAction(features, featureCount))
    traceCount = 0;

  memcpy(prevFeatures, features, featureCount * sizeof(uint32_t));
  prevFeatureCount = featureCount;
  prevAction = action;
  prevPhi = phi;
  st.lastAction = (int8_t)(action - ACT_NEUTRAL);
  st.uCommand = actionU(action);

  stepsInEpisode += 1.0f;
  if (stepsInEpisode * RL_DT >= episodeLimitS) {
    endEpisode();
    beginPause();
    newEpisode = true;
  }
  return st.uCommand;
}

void QLearning::endEpisode() {
  if (!greedyMode) {
    ++st.episode;
    if (st.episodeReward > st.bestReward)
      st.bestReward = st.episodeReward;
    st.epsilon = fmaxf(QL_EPS_MIN, st.epsilon * QL_EPS_DECAY);
    st.learningRate =
        fmaxf(QL_LR_MIN, st.learningRate * QL_LR_DECAY);
  }
  st.episodeReward = 0.0f;
  stepsInEpisode = 0.0f;
  episodeLimitS = QL_EPISODE_S;
  firstUpSeen = false;
  afterUpArmed = false;
  upHoldTime = 0.0f;
  prevFeatureCount = 0;
  prevPhi = 0.0f;
  exploreHold = 0;
  traceCount = 0;
}

bool QLearning::isPaused() { return paused; }
const QLearning::Stats& QLearning::stats() { return st; }
float* QLearning::table() { return weights; }
size_t QLearning::tableCount() { return WEIGHT_COUNT; }

void QLearning::getPersistState(PersistState &out) {
  out.episode = st.episode;
  out.topExploreSteps = st.topExploreSteps;
  out.epsilon = st.epsilon;
  out.epsilonTop = st.epsilonTop;
  out.learningRate = st.learningRate;
  out.bestReward = st.bestReward;
}

bool QLearning::restoreTrainingState(const PersistState &in) {
  if (!isfinite(in.epsilon) || !isfinite(in.epsilonTop) ||
      !isfinite(in.learningRate) || !isfinite(in.bestReward) ||
      in.epsilon < 0.0f || in.epsilon > 1.0f ||
      in.epsilonTop < 0.0f || in.epsilonTop > 1.0f ||
      in.learningRate < 0.0f || in.learningRate > 1.0f)
    return false;
  st.episode = in.episode;
  st.topExploreSteps = in.topExploreSteps;
  st.epsilon = in.epsilon;
  st.epsilonTop = in.epsilonTop;
  st.learningRate = in.learningRate;
  st.bestReward = in.bestReward;
  return true;
}
