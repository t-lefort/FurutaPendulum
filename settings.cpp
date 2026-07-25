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
  { "DutySlew", "/s",  &Data::dutySlew,       0.5f, 20.0f, 0.5f,   1 },
  { "K_alpha",  "",    &Data::kAlpha,         0.0f, 40.0f, 0.5f,   1 },
  { "K_adot",   "",    &Data::kAdot,          0.0f,  5.0f, 0.05f,  2 },
  { "K_th",     "",    &Data::kTh,            0.0f,  2.0f, 0.01f,  2 },
  { "K_thd",    "",    &Data::kThd,           0.0f,  2.0f, 0.01f,  2 },
  { "K_thi",    "",    &Data::kThi,           0.0f,  0.50f,0.005f, 3 },
  { "Ke_swing", "",    &Data::keSwing,     -200.0f,200.0f, 5.0f,   0 }, // signe a inverser si ca amortit
  { "Kthd_sw",  "",    &Data::kthdSwing,      0.0f, 0.05f, 0.001f, 3 },
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
static constexpr uint16_t EE_VERSION = 4;
static constexpr int      EE_ADDR    = 0;

static uint32_t checksum(const Data &d) {
  const uint8_t *p = (const uint8_t*)&d;
  uint32_t s = 2166136261u;               // FNV-1a
  for (size_t i = 0; i < sizeof(Data); i++) { s ^= p[i]; s *= 16777619u; }
  return s;
}

void loadDefaults() {
  cfg.armSign       = ARM_SIGN;
  cfg.pendSign      = PEND_SIGN;
  cfg.dutyLimit     = DUTY_LIMIT;
  cfg.dutySlew      = DUTY_SLEW_PER_S;
  cfg.kAlpha        = K_ALPHA;
  cfg.kAdot         = K_ADOT;
  cfg.kTh           = K_TH;
  cfg.kThd          = K_THD;
  cfg.kThi          = K_TH_I;
  cfg.keSwing       = KE_SWING;
  cfg.kthdSwing     = KTHD_SWING;
  cfg.pendMass      = PEND_MASS;
  cfg.pendLcom      = PEND_LCOM;
  cfg.pendLen       = PEND_LEN;
  cfg.alphaDotMax   = ALPHA_DOT_MAX;
  cfg.thetaDotMax   = THETA_DOT_MAX;
  cfg.thetaTurnsMax = THETA_TURNS_MAX;
}

bool save() {
  Header h = { EE_MAGIC, EE_VERSION, (uint16_t)sizeof(Data), checksum(cfg) };
  EEPROM.put(EE_ADDR, h);
  EEPROM.put(EE_ADDR + (int)sizeof(Header), cfg);
  return true;
}

bool load() {
  Header h;
  EEPROM.get(EE_ADDR, h);
  if (h.magic != EE_MAGIC || h.version != EE_VERSION || h.size != sizeof(Data))
    return false;
  Data tmp;
  EEPROM.get(EE_ADDR + (int)sizeof(Header), tmp);
  if (checksum(tmp) != h.sum) return false;
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
