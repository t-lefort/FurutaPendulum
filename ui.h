#pragma once
#include "config.h"

namespace UI {
  enum Event  : uint8_t { EV_NONE, EV_UP, EV_DOWN, EV_CLICK, EV_LONG };
  enum MenuId : uint8_t { M_MAIN, M_CLASSIC, M_QL, M_DEBUG };

  void  begin();
  void  invalidate();                // force un rafraîchissement complet
  Event poll();                      // à appeler très souvent depuis loop()

  void   setMenu(MenuId id);
  MenuId currentMenu();
  int    menuIndex();
  void   menuMove(int delta);

  // Rendu (10 Hz max, depuis loop() uniquement)
  void drawMenu(bool sdOk);
  void drawClassic(const PendulumState &s, uint8_t phase, bool balanceOnly);
  void drawQLearn(const PendulumState &s, uint32_t ep, float epsR, float epR,
                  float bestR, int8_t action, bool greedy);
  void drawMotorTest(const PendulumState &s);
  void drawOpenLoop(float wCmd, const PendulumState &s);
  void drawDebugAngles(const PendulumState &s, int32_t rawArm, int32_t rawPend);
  void drawAutoTest(uint8_t phase, float dPlus, float dMinus);
  void drawFault(uint8_t code);
  void drawMessage(const char *msg);

  // --- Éditeur de réglages (état ST_SETTINGS) ---
  void settingsReset();          // remet la sélection en tête, sort du mode édition
  bool settingsInput(Event ev);  // gère un événement ; retourne true = sortir vers IDLE
  void drawSettings();           // rendu (10 Hz, depuis loop())
}
