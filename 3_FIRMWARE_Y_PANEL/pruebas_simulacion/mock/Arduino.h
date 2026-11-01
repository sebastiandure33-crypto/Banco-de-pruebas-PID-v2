// Arduino simulado para probar el firmware en la PC (no se carga en la placa).
#pragma once
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include <deque>

#define PROGMEM
#define memcpy_P memcpy
class __FlashStringHelper;
#define F(s) (reinterpret_cast<const __FlashStringHelper*>(s))
#define OUTPUT 1
#define INPUT 0
const uint8_t A0 = 14, A1 = 15;

uint32_t millis();
void delay(unsigned long ms);
int analogRead(uint8_t pin);
void analogWrite(uint8_t pin, int v);
void pinMode(uint8_t, uint8_t);

class SimSerial {
 public:
  void begin(long) {}
  int available();
  int read();
  void write_str(const std::string& s);
  void print(const char* s) { write_str(s); }
  void print(const __FlashStringHelper* s) { write_str(reinterpret_cast<const char*>(s)); }
  void print(char c) { write_str(std::string(1, c)); }
  void print(unsigned char v) { write_str(std::to_string((unsigned)v)); }
  void print(int v) { write_str(std::to_string(v)); }
  void print(unsigned int v) { write_str(std::to_string(v)); }
  void print(long v) { write_str(std::to_string(v)); }
  void print(unsigned long v) { write_str(std::to_string(v)); }
  void print(double v, int d = 2) {
    char b[48];
    if (isnan(v)) { write_str("nan"); return; }
    snprintf(b, sizeof b, "%.*f", d, v);
    write_str(b);
  }
  template <typename T> void println(T v) { print(v); write_str("\r\n"); }
  void println(double v, int d) { print(v, d); write_str("\r\n"); }
  void println() { write_str("\r\n"); }
};
extern SimSerial Serial;

// min/max de Arduino (como funciones para no chocar con std::min/std::max)
template <class A, class B> inline auto max(A a, B b) -> decltype(a + b) { return a > b ? a : b; }
template <class A, class B> inline auto min(A a, B b) -> decltype(a + b) { return a < b ? a : b; }
