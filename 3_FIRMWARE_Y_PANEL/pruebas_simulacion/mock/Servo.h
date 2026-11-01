#pragma once
#include "Arduino.h"
void sim_servo_us(int us);
class Servo {
 public:
  uint8_t attach(int, int = 544, int = 2400) { attached = true; return 0; }
  void writeMicroseconds(int us) { sim_servo_us(us); }
  void detach() { attached = false; }
  bool attached = false;
};
