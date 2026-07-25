#pragma once
#include "config.h"

// ============================================================
//  Réglages runtime (RAM), éditables au menu et persistés en EEPROM.
//  Les constantes de config.h servent de VALEURS PAR DÉFAUT ; la valeur
//  réellement utilisée par le firmware est Settings::cfg.<champ>.
//  Lecture concurrente par l'ISR 1 kHz : chaque champ est un float aligné
//  (écriture atomique sur Cortex-M7) ; le chargement EEPROM se fait sous
//  noInterrupts() car il réécrit toute la structure d'un coup.
// ============================================================

namespace Settings {

  struct Data {
    // Signes (±1)
    float armSign, pendSign;
    // Moteur / alim
    float dutyLimit, dutySlew;
    // Équilibre (retour d'état) ; kThi = integrale sur theta (anti-frottement)
    float kAlpha, kAdot, kTh, kThd, kThi;
    // Swing-up (énergie)
    float keSwing, kthdSwing;
    // PI vitesse bras (Q-learning)
    float kpVel, kiVel;
    // Modèle physique du pendule
    float pendMass, pendLcom, pendLen;
    // Sécurité
    float alphaDotMax, thetaDotMax, thetaTurnsMax;

    // Dérivés (recalculés à partir du modèle physique)
    float eTop() const { return pendMass * G_GRAV * pendLcom; }
    float pendJ() const { return pendMass * pendLen * pendLen / 3.0f; }
  };

  extern Data cfg;

  void begin();          // charge l'EEPROM si valide, sinon les défauts (setup)
  void loadDefaults();   // remet les constantes de config.h
  bool save();           // écrit cfg en EEPROM (magic+version+checksum)
  bool load();           // relit l'EEPROM dans cfg (sous noInterrupts)

  // --- Introspection pour l'éditeur de menu ---
  int         count();                 // nombre de paramètres éditables
  const char* name(int i);
  const char* unit(int i);
  uint8_t     decimals(int i);
  float       get(int i);
  void        stepValue(int i, int dir); // ajoute ±pas et borne (dir = +1/-1)
}
