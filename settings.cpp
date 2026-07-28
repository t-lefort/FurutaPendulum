#include "settings.h"
#include <EEPROM.h>

namespace Settings {

Data cfg;

// ---- Table descriptive des paramètres éditables ----
// Les pointeurs visent les champs de l'instance globale cfg (adresses stables).
struct Desc {
  const char *name;
  const char *unit;
  float Data::*field;   // pointeur-sur-membre
  float minv, maxv, step;
  uint8_t decimals;
};

static const Desc TABLE[] = {
  { "ArmSign",  "",    &Data::armSign,       -1.0f,  1.0f, 2.0f,   0 }, // pas=2 -> bascule ±1
  { "PendSign", "",    &Data::pendSign,      -1.0f,  1.0f, 2.0f,   0 },
  { "DutyLim",  "",    &Data::dutyLimit,      0.05f, 0.90f, 0.05f, 2 },
  { "DutySlew", "/s",  &Data::dutySlew,       0.5f,100.0f, 2.0f,   1 },
  { "K_alpha",  "",    &Data::kAlpha,         0.0f, 40.0f, 0.5f,   1 },
  { "K_adot",   "",    &Data::kAdot,          0.0f,  5.0f, 0.05f,  2 },
  { "K_th",     "",    &Data::kTh,            0.0f,  2.0f, 0.01f,  2 },
  { "K_thd",    "",    &Data::kThd,           0.0f,  2.0f, 0.01f,  2 },
  { "K_thi",    "",    &Data::kThi,           0.0f,  0.50f,0.005f, 3 },
  { "Ke_swing", "",    &Data::keSwing,     -200.0f,200.0f, 5.0f,   0 }, // signe a inverser si ca amortit
  { "Kthd_sw",  "",    &Data::kthdSwing,      0.0f, 0.05f, 0.001f, 3 },
  { "QL_Umin",  "",    &Data::qlUMin,         0.0f, 0.90f, 0.01f,  2 }, // seuil de decollement
  { "QL_Umax",  "",    &Data::qlUMax,         0.05f,0.90f, 0.05f,  2 },
  { "PendMass", "kg",  &Data::pendMass,       0.005f,1.0f, 0.005f, 3 },
  { "PendLcom", "m",   &Data::pendLcom,       0.01f, 1.0f, 0.005f, 3 },
  { "PendLen",  "m",   &Data::pendLen,        0.01f, 1.5f, 0.005f, 3 },
  { "AdotMax",  "r/s", &Data::alphaDotMax,    5.0f, 80.0f, 1.0f,   0 },
  { "TdotMax",  "r/s", &Data::thetaDotMax,    5.0f, 80.0f, 1.0f,   0 },
  { "TurnsMax", "tr",  &Data::thetaTurnsMax,  0.0f, 10.0f, 0.5f,   1 },
};
static constexpr int N_PARAM = sizeof(TABLE) / sizeof(TABLE[0]);

// ---- EEPROM ----
struct Header { uint32_t magic; uint16_t version; uint16_t size; uint32_t sum; };
static constexpr uint32_t EE_MAGIC   = 0x46505354; // 'FPST'
// v4 : suppression de kpVel/kiVel (le Q-learning commande le couple
// directement, plus de boucle de vitesse). La taille de Data change, donc le
// controle 'h.size != sizeof(Data)' de load() rejette automatiquement
// l'ancienne sauvegarde -> retour aux defauts de config.h.
// v5 : ajout de qlUMin/qlUMax (bornes du jeu d'actions du Q-learning), placés
// EN FIN de struct -> load() relit une sauvegarde v4 comme un préfixe et les
// réglages déjà faits sur machine survivent à la mise à jour.
static constexpr uint16_t EE_VERSION = 5;
static constexpr int      EE_ADDR    = 0;

static uint32_t checksum(const void *p, size_t n) {
  const uint8_t *b = (const uint8_t*)p;
  uint32_t s = 2166136261u;               // FNV-1a
  for (size_t i = 0; i < n; i++) { s ^= b[i]; s *= 16777619u; }
  return s;
}

static void defaults(Data &d) {
  d.armSign       = ARM_SIGN;
  d.pendSign      = PEND_SIGN;
  d.dutyLimit     = DUTY_LIMIT;
  d.dutySlew      = DUTY_SLEW_PER_S;
  d.kAlpha        = K_ALPHA;
  d.kAdot         = K_ADOT;
  d.kTh           = K_TH;
  d.kThd          = K_THD;
  d.kThi          = K_TH_I;
  d.keSwing       = KE_SWING;
  d.kthdSwing     = KTHD_SWING;
  d.pendMass      = PEND_MASS;
  d.pendLcom      = PEND_LCOM;
  d.pendLen       = PEND_LEN;
  d.alphaDotMax   = ALPHA_DOT_MAX;
  d.thetaDotMax   = THETA_DOT_MAX;
  d.thetaTurnsMax = THETA_TURNS_MAX;
  d.qlUMin        = QL_U_MIN;
  d.qlUMax        = QL_U_MAX;
}

void loadDefaults() { defaults(cfg); }

bool save() {
  Header h = { EE_MAGIC, EE_VERSION, (uint16_t)sizeof(Data),
               checksum(&cfg, sizeof(Data)) };
  EEPROM.put(EE_ADDR, h);
  EEPROM.put(EE_ADDR + (int)sizeof(Header), cfg);
  return true;
}

bool load() {
  Header h;
  EEPROM.get(EE_ADDR, h);
  if (h.magic != EE_MAGIC) return false;
  // Écrite par un firmware PLUS RÉCENT : on ne sait pas ce qu'il y a dedans.
  if (h.version > EE_VERSION) return false;
  // Sauvegarde d'une version antérieure : elle est plus courte, mais c'est un
  // PRÉFIXE exact de la struct actuelle (les champs ne sont qu'ajoutés en fin).
  // On la relit telle quelle et les champs apparus depuis gardent leur défaut,
  // au lieu de tout jeter et de faire reperdre son réglage à l'utilisateur.
  if (h.size == 0 || h.size > sizeof(Data)) return false;

  uint8_t buf[sizeof(Data)];
  for (size_t i = 0; i < h.size; i++)
    buf[i] = EEPROM.read(EE_ADDR + (int)sizeof(Header) + (int)i);
  if (checksum(buf, h.size) != h.sum) return false;

  Data tmp;
  defaults(tmp);                     // champs absents de l'EEPROM = défauts
  memcpy(&tmp, buf, h.size);
  noInterrupts();          // réécriture atomique vis-à-vis de l'ISR 1 kHz
  cfg = tmp;
  interrupts();
  return true;
}

void begin() {
  loadDefaults();
  load();                  // écrase par l'EEPROM si présente et valide
}

// ---- Introspection éditeur ----
int         count()            { return N_PARAM; }
const char* name(int i)        { return TABLE[i].name; }
const char* unit(int i)        { return TABLE[i].unit; }
uint8_t     decimals(int i)    { return TABLE[i].decimals; }
float       get(int i)         { return cfg.*(TABLE[i].field); }

void stepValue(int i, int dir) {
  const Desc &d = TABLE[i];
  float v = cfg.*(d.field) + (float)dir * d.step;
  cfg.*(d.field) = constrain(v, d.minv, d.maxv);
}

} // namespace Settings
