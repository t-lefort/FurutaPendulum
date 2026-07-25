#include "ui.h"
#include "settings.h"
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

static Adafruit_GC9A01A tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_RST);

static bool fullRedraw = true;
void UI::invalidate() { fullRedraw = true; }

// ================= Sous-menus =================
struct MenuDef { const char *title; const char *const *items; uint8_t count; };

static const char *IT_MAIN[]    = { "Classic      >", "Q-Learning   >",
                                    "Debug        >", "Reglages     >",
                                    "Recalibrer bas" };
static const char *IT_CLASSIC[] = { "< Retour", "Swing-up", "Balance seul" };
static const char *IT_QL[]      = { "< Retour", "Entrainer", "Greedy",
                                    "Sauver table", "Charger table", "Reset table" };
static const char *IT_DEBUG[]   = { "< Retour", "Angles live",
                                    "Test mot. auto", "Jog manuel",
                                    "Openloop" };

static const MenuDef MENUS[] = {
  { "MENU",    IT_MAIN,    5 },
  { "CLASSIC", IT_CLASSIC, 3 },
  { "Q-LEARN", IT_QL,      6 },
  { "DEBUG",   IT_DEBUG,   5 },
};

static UI::MenuId menuId = UI::M_MAIN;
static int menuIdx = 0;

void UI::setMenu(MenuId id) { menuId = id; menuIdx = 0; fullRedraw = true; }
UI::MenuId UI::currentMenu() { return menuId; }
int  UI::menuIndex() { return menuIdx; }
void UI::menuMove(int d) {
  const int n = MENUS[menuId].count;
  menuIdx = (menuIdx + d % n + n) % n;
}

// ================= Encodeur rotatif (par interruptions) =================
// Décodage quadrature complet sur CLK+DT : aucun cran perdu, même si loop()
// est occupé par un rafraîchissement d'écran.
static volatile int16_t rotDelta = 0;

static void rotIsr() {
  static uint8_t prevAB = 3;                  // repos = CLK=DT=1 (pull-ups)
  static int8_t  acc    = 0;
  static const int8_t TBL[16] = { 0,-1, 1, 0,  1, 0, 0,-1,
                                 -1, 0, 0, 1,  0, 1,-1, 0 };
  const uint8_t ab = (digitalReadFast(PIN_UI_CLK) << 1) | digitalReadFast(PIN_UI_DT);
  acc += TBL[(prevAB << 2) | ab];
  prevAB = ab;
  if (ab == 3) {                              // position de cran (détent)
    if      (acc >=  2) rotDelta++;
    else if (acc <= -2) rotDelta--;
    acc = 0;
  }
}

// ================= Bouton =================
static uint8_t  lastSw = 1;
static uint32_t swDownMs = 0;
static bool     longSent = false;

void UI::begin() {
  pinMode(PIN_UI_CLK, INPUT_PULLUP);
  pinMode(PIN_UI_DT,  INPUT_PULLUP);
  pinMode(PIN_UI_SW,  INPUT_PULLUP);
  lastSw = digitalRead(PIN_UI_SW);
  attachInterrupt(digitalPinToInterrupt(PIN_UI_CLK), rotIsr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_UI_DT),  rotIsr, CHANGE);

  tft.begin();
  tft.setRotation(2);   // 0/2 = portrait (2 = retourné 180°), 1/3 = paysage
  tft.fillScreen(GC9A01A_BLACK);
  tft.setTextWrap(false);
}

UI::Event UI::poll() {
  // Rotation : on consomme un cran par appel (loop tourne bien plus vite)
  noInterrupts();
  int16_t d = rotDelta;
  if (d > 0) rotDelta--;
  else if (d < 0) rotDelta++;
  interrupts();
  if (d > 0) return EV_UP;
  if (d < 0) return EV_DOWN;

  // Bouton : clic court / appui long (>800 ms)
  const uint8_t sw = digitalRead(PIN_UI_SW);
  if (sw == 0 && lastSw == 1) { swDownMs = millis(); longSent = false; }
  if (sw == 0 && !longSent && millis() - swDownMs > 800) {
    longSent = true;
    lastSw = sw;
    return EV_LONG;
  }
  if (sw == 1 && lastSw == 0) {
    lastSw = sw;
    if (!longSent && millis() - swDownMs > 20) return EV_CLICK;
  }
  lastSw = sw;
  return EV_NONE;
}

// ================= Rendu =================
static void header(const char *title, uint16_t color) {
  tft.fillScreen(GC9A01A_BLACK);
  tft.setTextColor(color, GC9A01A_BLACK);
  tft.setTextSize(2);
  tft.setCursor(120 - 6 * strlen(title), 18);
  tft.print(title);
  tft.drawFastHLine(30, 40, 180, color);
}

// Écran ROND (Ø240) : le bord coupe les extrémités des lignes proches du haut/
// bas. On centre horizontalement (le centre est la partie la plus large) et on
// garde les pieds de page assez hauts (y <= ~208) et courts. Taille 1 = 6 px/car.
static void centerText(int y, const char *s, uint16_t fg = GC9A01A_LIGHTGREY,
                       uint16_t bg = GC9A01A_BLACK) {
  tft.setTextSize(1);
  tft.setTextColor(fg, bg);
  tft.setCursor(120 - 3 * (int)strlen(s), y);
  tft.print(s);
}

// Menu sans clignotement : le fond n'est effacé qu'au changement de menu ;
// les lignes sont réécrites en largeur fixe (le fond du texte recouvre l'ancien).
void UI::drawMenu(bool sdOk) {
  const MenuDef &m = MENUS[menuId];
  if (fullRedraw) {
    header(m.title, GC9A01A_CYAN);
    centerText(208, sdOk ? "SD OK" : "SD absente",
               sdOk ? GC9A01A_GREEN : GC9A01A_RED);
    fullRedraw = false;
  }
  tft.setTextSize(2);
  const int first = constrain(menuIdx - 2, 0, max(0, m.count - 5));
  for (int i = 0; i < 5; i++) {
    const int y = 56 + i * 30;
    char buf[20];
    if (first + i < m.count) {
      const bool sel = (first + i == menuIdx);
      snprintf(buf, sizeof(buf), "%c%-14.14s", sel ? '>' : ' ', m.items[first + i]);
      tft.setTextColor(sel ? GC9A01A_BLACK : GC9A01A_WHITE,
                       sel ? GC9A01A_CYAN  : GC9A01A_BLACK);
    } else {
      snprintf(buf, sizeof(buf), "%-15s", "");
      tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
    }
    tft.setCursor(26, y);
    tft.print(buf);
  }
}

static void liveValue(int y, const char *label, float v, const char *unit) {
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
  tft.setCursor(35, y);
  tft.print(label);
  tft.print(v, 1);
  tft.print(unit);
  tft.print("   ");
}

static void footerStop() {
  centerText(206, "clic = STOP");
}

void UI::drawClassic(const PendulumState &s, uint8_t phase, bool balanceOnly) {
  static uint8_t lastPhase = 255;
  if (fullRedraw || phase != lastPhase) {
    if (fullRedraw) { header(balanceOnly ? "BALANCE" : "CLASSIC", GC9A01A_YELLOW); footerStop(); }
    fullRedraw = false;
    lastPhase = phase;
    tft.setTextSize(2);
    tft.setTextColor(phase == 1 ? GC9A01A_GREEN : GC9A01A_ORANGE, GC9A01A_BLACK);
    tft.setCursor(55, 50);
    tft.print(phase == 1 ? "EQUILIBRE" : "SWING-UP ");
  }
  liveValue(80,  "a:  ", degrees(s.alpha), " deg");
  liveValue(105, "ad: ", s.alphaDot, "");
  liveValue(130, "td: ", s.thetaDot, "");
  liveValue(155, "u:  ", s.duty * 100.0f, " %");
  // Position du bras en TOURS : permet de voir la dérive approcher TurnsMax
  // (au-delà -> faute "plage bras" et arrêt moteur).
  liveValue(180, "th: ", s.theta / (float)TWO_PI, " tr");
}

void UI::drawQLearn(const PendulumState &s, uint32_t ep, float epsR, float epR,
                    float bestR, int8_t action, bool greedy) {
  if (fullRedraw) {
    header(greedy ? "QL GREEDY" : "QL TRAIN", GC9A01A_MAGENTA);
    footerStop();
    fullRedraw = false;
  }
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
  tft.setCursor(35, 50);  tft.printf("Ep:%4lu e:%.2f ", ep, epsR);
  tft.setCursor(35, 75);  tft.printf("R: %8.1f   ", epR);
  tft.setCursor(35, 100); tft.printf("Best:%7.1f ", bestR);
  tft.setCursor(35, 125); tft.printf("a: %+d  ", action);
  liveValue(150, "a:  ", degrees(s.alpha), " deg");
}

void UI::drawMotorTest(const PendulumState &s) {
  if (fullRedraw) {
    header("JOG", GC9A01A_ORANGE);
    centerText(194, "rotation = duty +/-5%");
    centerText(206, "clic = STOP");
    fullRedraw = false;
  }
  liveValue(70,  "th: ", degrees(s.theta), " deg");
  liveValue(100, "a:  ", degrees(s.alpha), " deg");
  liveValue(130, "u:  ", s.duty * 100.0f, " %");
}

// Test au banc sans capteur. On affiche AUSSI theta/theta_dot mesurés : si
// l'encodeur est accouplé, ils doivent suivre la consigne — c'est le moyen de
// vérifier le sens et le rapport de réduction une fois le moteur monté.
void UI::drawOpenLoop(float wCmd, const PendulumState &s) {
  if (fullRedraw) {
    header("OPENLOOP", GC9A01A_ORANGE);
    centerText(194, "rotation = vitesse");
    centerText(206, "clic = STOP");
    fullRedraw = false;
  }
  liveValue(80,  "w:  ", wCmd, " r/s");
  liveValue(115, "td: ", s.thetaDot, "");
  liveValue(150, "th: ", degrees(s.theta), " deg");
}

void UI::drawDebugAngles(const PendulumState &s, int32_t rawArm, int32_t rawPend) {
  if (fullRedraw) {
    header("ANGLES", GC9A01A_GREENYELLOW);
    centerText(194, "bras: 8000 cts/tour");
    centerText(206, "pend: 4000 cts/tour");
    fullRedraw = false;
  }
  liveValue(52,  "th: ", degrees(s.theta), " deg");
  liveValue(77,  "a:  ", degrees(s.alpha), " deg");
  liveValue(102, "td: ", s.thetaDot, "");
  liveValue(127, "ad: ", s.alphaDot, "");
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_CYAN, GC9A01A_BLACK);
  tft.setCursor(35, 152); tft.printf("cB:%8ld ", rawArm);
  tft.setCursor(35, 177); tft.printf("cP:%8ld ", rawPend);
}

void UI::drawAutoTest(uint8_t phase, float dPlus, float dMinus) {
  if (fullRedraw) {
    header("TEST AUTO", GC9A01A_ORANGE);
    footerStop();
    fullRedraw = false;
  }
  static const char *PH[] = { "init...   ", "sens +    ", "pause     ",
                              "sens -    ", "TERMINE   " };
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_YELLOW, GC9A01A_BLACK);
  tft.setCursor(60, 60);
  tft.print(PH[min((int)phase, 4)]);
  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
  tft.setCursor(35, 100); tft.printf("d+: %7.1f deg ", degrees(dPlus));
  tft.setCursor(35, 125); tft.printf("d-: %7.1f deg ", degrees(dMinus));
  if (phase >= 4) {
    tft.setTextSize(1);
    tft.setTextColor(GC9A01A_LIGHTGREY, GC9A01A_BLACK);
    tft.setCursor(30, 160);
    tft.print("attendu: d+ > 0 et d- < 0");
    tft.setCursor(30, 172);
    tft.print("sinon inverser ARM_SIGN");
  }
}

void UI::drawFault(uint8_t code) {
  static const char *NAMES[] = { "aucun", "alpha_dot", "theta_dot",
                                 "plage bras", "saturation", "stop" };
  if (!fullRedraw) return;
  fullRedraw = false;
  header("FAULT", GC9A01A_RED);
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_RED, GC9A01A_BLACK);
  tft.setCursor(45, 100);
  tft.print(code < 6 ? NAMES[code] : "?");
  centerText(206, "clic = retour menu");
}

void UI::drawMessage(const char *msg) {
  header("INFO", GC9A01A_CYAN);
  tft.setTextSize(2);
  tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
  tft.setCursor(120 - 6 * strlen(msg), 105);
  tft.print(msg);
  fullRedraw = true;   // l'écran suivant repartira d'un fond propre
}

// ================= Éditeur de réglages =================
// Lignes = N paramètres (settings.*) suivis de 4 actions.
static const char *SET_ACTIONS[] = { "Sauver EEPROM", "Charger EEPROM",
                                     "Defauts", "< Retour" };
static constexpr int N_ACTIONS = 4;

static int  setSel     = 0;      // ligne sélectionnée
static bool setEditing = false;  // true = rotation modifie la valeur
static const char *setStatus  = nullptr;
static uint32_t    setStatusMs = 0;

static void flashStatus(const char *s) { setStatus = s; setStatusMs = millis(); }

void UI::settingsReset() {
  setSel = 0; setEditing = false; setStatus = nullptr; fullRedraw = true;
}

bool UI::settingsInput(Event ev) {
  const int nP = Settings::count();
  const int total = nP + N_ACTIONS;

  if (setEditing) {
    if (ev == EV_UP)   Settings::stepValue(setSel, +1);
    if (ev == EV_DOWN) Settings::stepValue(setSel, -1);
    if (ev == EV_CLICK || ev == EV_LONG) setEditing = false;
    return false;
  }

  switch (ev) {
    case EV_UP:   if (setSel < total - 1) setSel++; break;
    case EV_DOWN: if (setSel > 0)         setSel--; break;
    case EV_LONG: return true;                       // sortie
    case EV_CLICK:
      if (setSel < nP) {
        setEditing = true;                           // édite le paramètre
      } else {
        switch (setSel - nP) {
          case 0: flashStatus(Settings::save() ? "Sauve EEPROM" : "Err EEPROM"); break;
          case 1: flashStatus(Settings::load() ? "Charge EEPROM" : "Rien/invalide"); break;
          case 2: Settings::loadDefaults(); flashStatus("Defauts charges"); break;
          default: return true;                      // < Retour
        }
      }
      break;
    default: break;
  }
  return false;
}

void UI::drawSettings() {
  if (fullRedraw) { header("REGLAGES", GC9A01A_CYAN); fullRedraw = false; }

  const int nP = Settings::count();
  const int total = nP + N_ACTIONS;
  const int first = constrain(setSel - 2, 0, max(0, total - 5));

  tft.setTextSize(2);
  for (int i = 0; i < 5; i++) {
    const int y   = 52 + i * 28;
    const int idx = first + i;
    char buf[24];

    if (idx >= total) {                              // ligne vide (padding)
      tft.setTextColor(GC9A01A_WHITE, GC9A01A_BLACK);
      snprintf(buf, sizeof(buf), "%-16s", "");
      tft.setCursor(20, y); tft.print(buf);
      continue;
    }

    const bool sel = (idx == setSel);
    // Couleurs : édition = vert, sélection = cyan, sinon blanc/noir
    const uint16_t fg = sel ? GC9A01A_BLACK : GC9A01A_WHITE;
    const uint16_t bg = sel ? (setEditing ? GC9A01A_GREEN : GC9A01A_CYAN)
                            : GC9A01A_BLACK;
    tft.setTextColor(fg, bg);

    if (idx < nP) {                                  // ligne paramètre
      char val[12];
      dtostrf(Settings::get(idx), 0, Settings::decimals(idx), val);
      snprintf(buf, sizeof(buf), "%c%-8.8s%6s", sel ? '>' : ' ',
               Settings::name(idx), val);
    } else {                                         // ligne action
      snprintf(buf, sizeof(buf), "%c%-15.15s", sel ? '>' : ' ',
               SET_ACTIONS[idx - nP]);
    }
    tft.setCursor(20, y); tft.print(buf);
  }

  // Bas d'écran (aide/état) : champ de largeur FIXE centré sur l'écran rond
  // (x=48, 24 car. -> 48..192) pour à la fois recouvrir l'ancien et rester
  // dans le cercle visible. Les libellés sont maintenus <= 24 caractères.
  tft.setTextSize(1);
  tft.setTextColor(GC9A01A_LIGHTGREY, GC9A01A_BLACK);
  tft.setCursor(48, 200);
  char tmp[28], hint[28];
  if (setStatus && millis() - setStatusMs < 1200) {
    strncpy(tmp, setStatus, sizeof(tmp));
  } else if (setEditing) {
    strncpy(tmp, "rotation=+/- clic=OK", sizeof(tmp));
  } else if (setSel < nP) {
    snprintf(tmp, sizeof(tmp), "unite:%s clic=editer", Settings::unit(setSel));
  } else {
    strncpy(tmp, "clic=OK  long=retour", sizeof(tmp));
  }
  tmp[sizeof(tmp) - 1] = '\0';
  snprintf(hint, sizeof(hint), "%-24.24s", tmp);   // largeur fixe = pas de résidu
  tft.print(hint);
}
