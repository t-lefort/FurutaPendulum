#include "qlearning.h"
#include "encoders.h"   // rezeroArm() au debut de chaque episode
#include "settings.h"   // qlUMin / qlUMax : bornes du jeu d'actions
using Settings::cfg;

// Table dans la RAM2 (DMAMEM) : 49*41*7 floats = ~56 kB, laisse la RAM1 au code
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
// Exploration persistante : pas restants sur l'action aleatoire en cours.
static int   exploreHold  = 0;
static int   exploreAct   = 0;

// En cas d'EGALITE (typiquement une table vierge, tout a zero), on doit
// retomber sur l'action NEUTRE (couple nul) et non sur l'indice 0, qui vaut
// -u_max : sinon un agent non entraine applique le couple maxi dans un sens en
// permanence et le bras part en toupie jusqu'a la faute "plage bras".
static constexpr int ACT_NEUTRAL = QL_N_ACT / 2;   // action de couple nul
static constexpr int QL_N_LVL    = ACT_NEUTRAL;    // niveaux non nuls par sens

// Actions = couples normalises appliques DIRECTEMENT au moteur.
// Les niveaux non nuls sont repartis sur [uMin, uMax] et non sur [0, uMax] :
// sous le seuil de decollement du train d'engrenages, une action ne produit
// AUCUN mouvement. Reparties depuis 0, les petites actions etaient des
// "ne rien faire" facturees au prix du malus |u| -> l'agent n'avait en pratique
// que 4 actions distinctes sur 7, et l'exploration en gaspillait 29 %.
// Bornes runtime (menu Reglages : QL_Umin / QL_Umax), le seuil se mesure.
static inline float actionU(int a) {
  const int k = a - ACT_NEUTRAL;                  // -QL_N_LVL .. +QL_N_LVL
  if (k == 0) return 0.0f;
  const float uMin = cfg.qlUMin;
  const float uMax = max(cfg.qlUMax, uMin);
  const float t = (QL_N_LVL > 1) ? (float)(abs(k) - 1) / (float)(QL_N_LVL - 1)
                                 : 1.0f;          // 0, 0.5, 1 pour 3 niveaux
  const float u = uMin + t * (uMax - uMin);
  return (k > 0) ? u : -u;
}

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
  // --- Base POSITIVE : 0 en bas, +2 en haut. ---
  // Elle valait 2*cos(alpha), donc -2 en bas. Deux consequences fatales :
  //  1) "ne rien faire" etait l'action la MOINS chere de toutes (tous les autres
  //     termes sont des malus, nuls a l'arret) -> optimum local parfait, l'agent
  //     se fige en bas. C'est le comportement observe en entrainement.
  //  2) V(rester en bas) = -2/(1-gamma) = -400, alors que l'etat terminal vaut
  //     r + QL_R_OUT_RANGE = -52 SANS bootstrap : terminer l'episode etait
  //     MEILLEUR que survivre. La penalite terminale recompensait le suicide.
  // Avec r >= 0, l'immobilite vaut 0, toute montee du pendule paie, et la
  // penalite terminale est enfin une penalite.
  float r = 1.0f + cosf(s.alpha);

  // --- Vitesse du pendule : penalisee UNIQUEMENT pres du haut ---
  // Un swing-up exige de passer au point bas a ~sqrt(4*m*g*lcom/J) rad/s
  // (~27 rad/s ici). Un malus permanent sur |alpha_dot| punissait donc l'agent
  // exactement au moment ou il fait la bonne chose : c'est l'energie meme dont
  // il a besoin. Pres du haut au contraire, arriver lentement est ce qu'on veut
  // (et cela empeche de "farmer" le bonus en traversant la verticale a fond).
  // /!\ Ce terme est le SEUL qui distingue "arrete en haut" de "traverse le haut
  // a pleine vitesse". Avec QL_K_ENERGY actif il doit etre du meme ordre, sinon
  // l'agent maximise l'energie en faisant TOURNER le pendule en continu (mesure
  // sim : E = eTop est satisfait aussi bien en rotation qu'a l'arret au sommet).
  if (fabsf(s.alpha) < QL_ADOT_TOP_RAD) r -= QL_K_ADOT_TOP * fabsf(s.alphaDot);

  // Cone de recompense pres du haut (cf. QL_K_BAL dans config.h) : sans lui la
  // recompense est PLATE dans la zone d'equilibre et rien ne guide le rattrapage.
  if (QL_K_BAL > 0.0f) {
    const float ar = s.alpha / QL_BAL_CONE_RAD;
    const float wr = s.alphaDot / QL_BAL_CONE_ADOT;
    r += QL_K_BAL * max(0.0f, 1.0f - ar * ar - wr * wr);
  }

  // --- Vitesse du bras : barriere douce seulement ---
  // theta_dot n'est PAS dans l'etat ([alpha, alpha_dot]) : un malus permanent
  // est inattribuable par l'agent, c'est un biais vers l'immobilite qu'il ne
  // peut pas apprendre a eviter. On ne penalise donc que l'approche de la
  // limite d'episode, ou le lien action -> penalite est net.
  const float tdOver = fabsf(s.thetaDot) - QL_TDOT_SOFT;
  if (tdOver > 0.0f) r -= QL_K_TDOT * tdOver;

  // --- Proximite ENERGETIQUE du sommet (terme DENSE) ---
  // Sans ce terme l'agent plafonne dans un optimum local tres net, mesure en
  // simulation : il pompe le pendule jusqu'a ~l'horizontale et s'y maintient.
  // Raison : 1+cos(alpha) recompense la HAUTEUR, pas l'ENERGIE. Se maintenir a
  // l'horizontale rapporte ~1,0 par pas indefiniment, alors qu'un vrai swing-up
  // impose de repasser par le bas (ou 1+cos = 0) pour accumuler de la vitesse.
  // Les bonus de sommet ne corrigent rien : a 20 Hz le pendule traverse la zone
  // +/-10 deg en ~9 ms quand il passe balistiquement, donc l'agent ne les
  // ECHANTILLONNE jamais — mesure : QL_R_BAL de 20 a 600 ne change RIEN, les
  // trajectoires restent identiques. Une recompense jamais percue n'a pas de
  // gradient. Il faut donc un terme dense, toujours vu, qui paie l'energie.
  // E = 1/2*J*alpha_dot^2 + m*g*lcom*cos(alpha), nulle en bas, eTop() en haut.
  // QL_K_ENERGY = 0 desactive le terme (comportement d'avant).
  if (QL_K_ENERGY > 0.0f) {
    const float eTop = max(cfg.eTop(), 1e-6f);
    const float e = 0.5f * cfg.pendJ() * s.alphaDot * s.alphaDot
                    + eTop * cosf(s.alpha);
    // Ecart relatif a l'energie du sommet, borne : reste POSITIF (cf. invariant
    // "base non negative") et sature au lieu de creuser un puits negatif.
    const float gap = fabsf(e - eTop) / (2.0f * eTop);
    r += QL_K_ENERGY * max(0.0f, 1.0f - gap);
  }

  // Cout de l'effort, faible : il doit departager, pas dominer.
  const float uMaxNow = max(cfg.qlUMax, 1e-3f);
  r -= 0.02f * fabsf(actionU(action)) / uMaxNow;

  // Les bonus "pendule en haut" exigent un bras LENT. Sans cette condition, un
  // bras qui tourne a fond maintient le pendule releve par effet centrifuge et
  // touche les bonus sans jamais equilibrer : c'est un optimum local tres
  // attractif dont l'agent ne ressort plus. NE PAS SUPPRIMER CE GATING.
  // /!\ Mais le seuil doit rester ATTEIGNABLE : a l'instant ou le pendule arrive
  // en haut, le bras tourne encore (mesure sim sur le swing-up classique :
  // 6 a 8 rad/s a l'arrivee). Avec un gate a 3 rad/s l'agent ne touchait JAMAIS
  // le bonus au moment de l'arrivee -> rien ne lui signalait que monter est bien,
  // et le swing-up ne s'apprenait pas. Regler par QL_UP_TDOT / QL_BAL_TDOT.
  if (fabsf(s.alpha) < QL_UP_RAD  && fabsf(s.thetaDot) < QL_UP_TDOT)  r += QL_R_UP;
  if (fabsf(s.alpha) < QL_BAL_RAD && fabsf(s.alphaDot) < QL_BAL_ADOT
                                  && fabsf(s.thetaDot) < QL_BAL_TDOT) r += QL_R_BAL;
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
  paused      = true;
  pauseTime   = 0.0f;
  exploreHold = 0;
}

// L'exploration persistante ne doit jamais enjamber une pause : le couple est
// coupe pendant celle-ci, tenir l'action n'aurait aucun sens.
static inline void beginPause() {
  paused = true; pauseTime = 0.0f; exploreHold = 0;
}

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

  // ---- Choix de l'action : epsilon-greedy a exploration PERSISTANTE ----
  // Une action aleatoire retiree a chaque pas est un bruit blanc a 20 Hz : de
  // moyenne nulle, il ne peut pas POMPER un oscillateur a ~1,5 Hz, qui demande
  // ~1/3 de seconde de couple dans le meme sens. L'agent ne rencontrait donc
  // jamais un debut de swing-up et ne pouvait pas l'apprendre. On TIENT donc
  // l'action tiree pendant QL_EXPLORE_HOLD pas (~200 ms).
  int a;
  if (greedyMode) {
    a = bestAction(sIdx);
  } else if (exploreHold > 0) {
    exploreHold--;
    a = exploreAct;
  } else if ((random(10000) / 10000.0f) < st.epsilon) {
    a = exploreAct = random(QL_N_ACT);
    exploreHold = QL_EXPLORE_HOLD - 1;
  } else {
    a = bestAction(sIdx);
  }

  prevStateIdx = sIdx;
  prevAction   = a;
  st.lastAction = (int8_t)(a - ACT_NEUTRAL);
  st.uCommand   = actionU(a);

  // Gestion de l'épisode
  stepsInEpisode += 1.0f;
  if (stepsInEpisode * RL_DT >= QL_EPISODE_S) {
    endEpisode();
    newEpisode = true;
    beginPause();       // moteur coupe, on laisse tout retomber
  }
  return actionU(a);
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
  exploreHold  = 0;
}

bool QLearning::isPaused() { return paused; }

const QLearning::Stats& QLearning::stats() { return st; }
float*  QLearning::table()      { return Q; }
size_t  QLearning::tableCount() { return (size_t)QL_N_ALPHA * QL_N_ADOT * QL_N_ACT; }
