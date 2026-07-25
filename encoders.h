#pragma once
#include "config.h"

// Décommente pour utiliser le décodeur quadrature MATÉRIEL du Teensy 4.1
// (bibliothèque "Teensy-4.x-Quad-Encoder-Library" de mjs513, à installer).
// Commenté : utilise la bibliothèque Encoder standard (interruptions), fournie
// avec Teensyduino — largement suffisante pour valider le montage.
//#define USE_HW_QUADENCODER

namespace Encoders {
  void begin();
  // Recale alpha : à appeler pendule IMMOBILE EN BAS (alpha = pi) et remet theta à 0
  void calibrateBottom();
  // À appeler à chaque tick de la boucle de contrôle (1 kHz)
  void update(PendulumState &s);
  // Angle de l'arbre MOTEUR (rad, non borné), pour la commutation FOC.
  // Basé sur le compteur brut : insensible à calibrateBottom().
  float motorShaftAngle();
  // Counts bruts (pour l'écran de debug)
  int32_t rawArm();
  int32_t rawPend();
}
