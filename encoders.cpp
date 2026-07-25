#include "encoders.h"
#include "settings.h"
using Settings::cfg;

#ifdef USE_HW_QUADENCODER
  #include "QuadEncoder.h"
  static QuadEncoder encArm (1, PIN_ENC_ARM_A,  PIN_ENC_ARM_B,  0);
  static QuadEncoder encPend(2, PIN_ENC_PEND_A, PIN_ENC_PEND_B, 0);
  static inline int32_t readArm()  { return encArm.read();  }
  static inline int32_t readPend() { return encPend.read(); }
  static inline void writeArm(int32_t v)  { encArm.write(v);  }
  static inline void writePend(int32_t v) { encPend.write(v); }
#else
  #include <Encoder.h>
  static Encoder encArm (PIN_ENC_ARM_A,  PIN_ENC_ARM_B);
  static Encoder encPend(PIN_ENC_PEND_A, PIN_ENC_PEND_B);
  static inline int32_t readArm()  { return encArm.read();  }
  static inline int32_t readPend() { return encPend.read(); }
  static inline void writeArm(int32_t v)  { encArm.write(v);  }
  static inline void writePend(int32_t v) { encPend.write(v); }
#endif

// Zéros de calibration en OFFSET LOGICIEL : on n'écrit JAMAIS les compteurs
// matériels, car la commutation FOC s'appuie sur le compteur brut du bras
// (le remettre à zéro détruirait le zéro électrique établi par initFOC()).
static int32_t armZero  = 0;
static int32_t pendZero = 0;

static float thetaPrev = 0.0f;
static float alphaPrev = (float)PI;
static float thetaDotF = 0.0f;
static float alphaDotF = 0.0f;

static inline float wrapPi(float a) {
  while (a >  PI) a -= TWO_PI;
  while (a < -PI) a += TWO_PI;
  return a;
}

void Encoders::begin() {
#ifdef USE_HW_QUADENCODER
  encArm.setInitConfig();  encArm.init();
  encPend.setInitConfig(); encPend.init();
#endif
  calibrateBottom();
}

void Encoders::calibrateBottom() {
  armZero  = readArm();    // offset seulement : compteurs matériels intacts
  pendZero = readPend();
  thetaPrev = 0.0f;
  alphaPrev = (float)PI;   // pendule en bas = pi (0 = en haut)
  thetaDotF = 0.0f;
  alphaDotF = 0.0f;
}

// Angle de l'arbre MOTEUR (rad, non borné) pour la commutation FOC.
// Utilise le compteur BRUT : indépendant de la calibration du bras.
float Encoders::motorShaftAngle() {
  return (float)readArm() * MOTOR_CNT2RAD;
}

int32_t Encoders::rawArm()  { return readArm();  }
int32_t Encoders::rawPend() { return readPend(); }

void Encoders::update(PendulumState &s) {
  const float theta = cfg.armSign  * (float)(readArm() - armZero) * ARM_CNT2RAD;
  // alpha brut : compte depuis la position basse -> décalage de pi
  const float alphaCont = cfg.pendSign * (float)(readPend() - pendZero) * PEND_CNT2RAD
                        + (float)PI;
  const float alpha = wrapPi(alphaCont);

  // Vitesses brutes (différence finie) puis filtre passe-bas exponentiel
  const float thetaDotRaw = (theta - thetaPrev) / CTRL_DT;
  const float alphaDotRaw = wrapPi(alpha - alphaPrev) / CTRL_DT;
  thetaPrev = theta;
  alphaPrev = alpha;

  thetaDotF += VEL_FILT_ALPHA * (thetaDotRaw - thetaDotF);
  alphaDotF += VEL_FILT_ALPHA * (alphaDotRaw - alphaDotF);

  s.theta    = theta;
  s.alpha    = alpha;
  s.thetaDot = thetaDotF;
  s.alphaDot = alphaDotF;
}
