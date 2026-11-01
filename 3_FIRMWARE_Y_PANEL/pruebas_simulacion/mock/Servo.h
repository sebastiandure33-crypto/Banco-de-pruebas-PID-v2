#pragma once
#include "Arduino.h"
void sim_servo_us(int pin, int us);
class Servo {
 public:
  uint8_t attach(int p, int = 544, int = 2400) { pin = p; attached = true; if (pend >= 0) sim_servo_us(pin, pend); return 0; }
  void writeMicroseconds(int us) { if (pin < 0) { pend = us; return; } sim_servo_us(pin, us); }
  void detach() { attached = false; }
  bool attached = false;
  int pin = -1, pend = -1;
};
