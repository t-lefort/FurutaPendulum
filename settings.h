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

  // /!\ ORDRE DES CHAMPS : uniquement AJOUTER EN FIN de struct, jamais insérer
  // ni réordonner. load() sait relire une sauvegarde EEPROM plus ancienne (donc
  // plus courte) en la traitant comme un PRÉFIXE de la struct actuelle, les
  // champs nouveaux gardant leur valeur par défaut — ce qui évite de perdre des
  // heures de réglage sur machine à chaque ajout de paramètre. Cette garantie
  // ne tient que si l'ordre existant n'est jamais touché. L'ordre du menu, lui,
  // est donné par TABLE[] dans settings.cpp : il est libre.
  struct Data {
    // Signes (±1)
    float armSign, pendSign;
    // Moteur / alim
    float dutyLimit, dutySlew;
    // Équilibre (retour d'état) ; kThi = integrale sur theta (anti-frottement)
    float kAlpha, kAdot, kTh, kThd, kThi;
    // Swing-up (énergie)
    float keSwing, kthdSwing;
    // Modèle physique du pendule
    float pendMass, pendLcom, pendLen;
    // Sécurité
    float alphaDotMax, thetaDotMax, thetaTurnsMax;
    // --- Ajouts v5 (en fin de struct, cf. note ci-dessus) ---
    // Q-learning : bornes du jeu d'actions (couples normalisés).
    // qlUMin = couple minimal qui met RÉELLEMENT le bras en mouvement (seuil de
    // décollement du train d'engrenages) ; en dessous, l'action ne fait rien.
    float qlUMin, qlUMax;

    // Dérivés (recalculés à partir du modèle physique)
    float eTop() const { return pendMass * G_GRAV * pendLcom; }
    float pendJ() const {
      if (PEND_J_ROD_BOB)
        return pendMass * pendLen * (4.0f * pendLcom - pendLen) / 3.0f;
      return pendMass * pendLen * pendLen / 3.0f;
    }
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
