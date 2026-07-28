#pragma once
#include "config.h"

// Moteur BLDC pilote en FOC (SimpleFOC) via un SimpleFOCMini + capteur AS5600.
//
// REPARTITION DES TACHES — important :
//   - setDuty/hardStop sont appeles par l'ISR 1 kHz et ne font QUE
//     mettre a jour une consigne (float aligne = ecriture atomique).
//   - spin() fait la commutation FOC et tourne dans SON PROPRE timer a
//     FOC_FREQ_HZ (10 kHz), pas dans loop(). C'est indispensable : loop() se
//     bloque des dizaines de ms (redraw SPI, SD, Serial, delay) et une
//     commutation interrompue fait saccader le moteur.
//     C'est possible parce que le capteur est l'encodeur du bras (lecture d'un
//     compteur, rapide et ISR-safe) et non un capteur I2C.
namespace Motor {
  void begin();          // init FOC — /!\ initFOC() FAIT BOUGER LE MOTEUR
  bool ready();          // false si le capteur/l'alignement a echoue

  // Consigne de COUPLE NORMALISE [-1, 1] (mise a l'echelle par MOTOR_VOLT_LIMIT),
  // avec limite et slew rate. À appeler à 1 kHz depuis l'ISR.
  void setDuty(float u);
  void setDuty(float u, float slewPerSecond);

  // Coupure immédiate (couple nul) — utilisée par la sécurité
  void hardStop();

  // Commutation FOC — appelée par le timer FOC (FOC_FREQ_HZ), jamais loop().
  void spin();

  // --- Mode OPEN-LOOP (test au banc, menu Debug) ---
  // N'utilise PAS le capteur : le champ tourne à vitesse imposée et le rotor
  // suit, comme un pas-à-pas. Fonctionne donc même si l'alignement FOC a
  // échoué ou si l'encodeur n'est pas encore accouplé au moteur.
  void  openLoopEnable(bool on);          // bascule open-loop <-> couple
  void  openLoopSetVelocity(float w);     // rad/s de l'arbre moteur

  float duty();          // couple normalisé réellement appliqué
  bool  isSaturated();   // true si |duty| == DUTY_LIMIT
}
