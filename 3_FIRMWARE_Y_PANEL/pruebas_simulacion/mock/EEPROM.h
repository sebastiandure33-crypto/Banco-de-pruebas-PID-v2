#pragma once
#include "Arduino.h"
extern uint8_t sim_eeprom[1024];
class SimEEPROM {
 public:
  template <typename T> T& get(int a, T& v) { memcpy(&v, sim_eeprom + a, sizeof(T)); return v; }
  template <typename T> const T& put(int a, const T& v) { memcpy(sim_eeprom + a, &v, sizeof(T)); return v; }
};
extern SimEEPROM EEPROM;
