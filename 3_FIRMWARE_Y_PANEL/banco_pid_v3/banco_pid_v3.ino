/*
  ============================================================================
  BANCO DE PRUEBAS PID v3  —  Arduino Nano (ATmega328P) + 2 servos SG90
  ============================================================================
  Un servo (D9) gira el brazo con 2 LDR para seguir una línea negra. Un segundo
  servo (D10, "perturbador") gira el sector donde está la línea: genera
  perturbaciones exactas y repetibles. El controlador es un PID configurable
  (P / PI / PD / PID) con interruptores para comparar cada mejora, ensayos
  guiados y métricas calculadas en la placa.

  CONEXIONES
    LDR izquierdo : 5V — LDR — A0 — R — GND
    LDR derecho   : 5V — LDR — A1 — R — GND
    Servo brazo   : señal D9   } alimentación externa (entrada -> C1||C2 -> salida),
    Servo línea   : señal D10  } masa común con el Nano
    LED (opcional): D5 / D6 con resistencia (si van fijos a 5V, ledi/ledd no actúan)

  MAGNITUDES (todas en grados del brazo)
    s  posición normalizada de la línea entre los LDR (−1..1)
    p  posición del brazo respecto de la línea = s / pendiente   (medida óptica)
    L  posición ordenada de la línea (perturbador), 0 = bajo el brazo en 90°
    e  error = ref − p
    u  salida del PID = ángulo pedido al servo del brazo = u0 + P + I + D

  PROTOCOLO SERIE (115200, una orden por línea)
    get | set <nom> <val> | run | stop | cal | calp | goto <g> | lin <g> | lcero
    ens <tipo> <A> [p2]   tipos: pesc resc sesc pram psen   | save | load | defaults
    tel 1|0 | status | ?
  Respuestas: OK ERR P K ST S T C CS CP E M BOOT END I  (ver LEEME_FIRMWARE.txt)
  ============================================================================
*/

#include <Arduino.h>
#include <Servo.h>
#include <EEPROM.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#if defined(__AVR__)
  #include <avr/pgmspace.h>
  #include <avr/wdt.h>
#endif

#define FW_VERSION "3.0"
#define USAR_WATCHDOG 0          // 1 solo con Optiboot (ver LEEME)

// ---------------------------------------------------------------- pines
const uint8_t PIN_LDR_I = A0;
const uint8_t PIN_LDR_D = A1;
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_PERT  = 10;
const uint8_t PIN_LED_I = 5;
const uint8_t PIN_LED_D = 6;

// ---------------------------------------------------------------- servo
const float US_CENTRO = 1472.0f;
const float US_POR_GRADO = (2400.0f - 544.0f) / 180.0f;

// ---------------------------------------------------------------- configuración
struct Config {
  uint16_t magic;
  uint8_t  version;
  // controlador
  float modo, kp, ki, kd, kdf, dmed, aw, zm, ksat, lvel, vmax, per, u0, busq;
  // servo del brazo
  float amin, amax, trim;
  // perturbador
  float pcen, prel, psig, plmax, pvel;
  // sensores
  float corr, gi, oi, gd, od, tau, vison, visoff, ledi, ledd;
  // ensayos / varios
  float tdur, tpre, vbusq, vcal, thold, teldiv;
  // calibración del brazo (cal)
  uint8_t calOk, calModo;
  float fBlancoI, fNegroI, fBlancoD, fNegroD;   // conductancias extremas
  float rawMin, rawMax;                         // extremos crudos comunes (modos 0 y 2)
  float pendiente;                              // ds/dθ del brazo
  float thLinea;
  // calibración del perturbador (calp)
  uint8_t calpOk;
  float pendLinea;                              // ds/dL
  uint16_t crc;
};
const uint16_t CFG_MAGIC = 0x5050;   // 'PP'
const uint8_t  CFG_VERSION = 3;
Config cfg;

// tipo: 0 real, 1 entero, 2 sí/no
struct ParamDef { char nombre[7]; uint8_t ofs; uint8_t tipo; float minv, maxv, def; };
#define PD(n, campo, t, a, b, d) {n, offsetof(Config, campo), t, a, b, d}
const ParamDef PARAMS[] PROGMEM = {
  PD("modo", modo, 1, 1, 4, 2.0f),     PD("kp", kp, 0, 0, 5, 0.4f),        PD("ki", ki, 0, 0, 30, 6.0f),
  PD("kd", kd, 0, 0, 1, 0.0f),         PD("kdf", kdf, 0, 0, 300, 40.0f),    PD("dmed", dmed, 2, 0, 1, 1.0f),
  PD("aw", aw, 2, 0, 1, 1.0f),         PD("zm", zm, 0, 0, 3, 0.2f),        PD("ksat", ksat, 0, 1, 10, 3.0f),
  PD("lvel", lvel, 2, 0, 1, 1.0f),     PD("vmax", vmax, 0, 10, 600, 150.0f), PD("per", per, 1, 10, 50, 20.0f),
  PD("u0", u0, 0, 60, 120, 90.0f),      PD("busq", busq, 2, 0, 1, 1.0f),
  PD("amin", amin, 0, 30, 89, 60.0f),   PD("amax", amax, 0, 91, 150, 120.0f), PD("trim", trim, 0, -30, 30, 0.0f),
  PD("pcen", pcen, 0, 0, 180, 90.0f),   PD("prel", prel, 0, 2, 15, 7.375f),   PD("psig", psig, 1, -1, 1, 1.0f),
  PD("plmax", plmax, 0, 1, 12.5f, 12.0f), PD("pvel", pvel, 0, 0, 200, 30.0f),
  PD("corr", corr, 1, 0, 2, 1.0f),     PD("gi", gi, 0, 0.2f, 5, 1.0f),     PD("oi", oi, 0, -600, 600, 0.0f),
  PD("gd", gd, 0, 0.2f, 5, 1.0f),      PD("od", od, 0, -600, 600, 0.0f),   PD("tau", tau, 0, 0, 500, 40.0f),
  PD("vison", vison, 0, 0.05f, 1.5f, 0.35f), PD("visoff", visoff, 0, 0, 1.2f, 0.2f),
  PD("ledi", ledi, 1, 0, 255, 255.0f),   PD("ledd", ledd, 1, 0, 255, 255.0f),
  PD("tdur", tdur, 0, 1, 20, 4.0f),    PD("tpre", tpre, 0, 0, 3, 0.5f),    PD("vbusq", vbusq, 0, 2, 200, 20.0f),
  PD("vcal", vcal, 0, 2, 30, 8.0f),    PD("thold", thold, 0, 0, 5000, 300.0f), PD("teldiv", teldiv, 1, 1, 50, 2.0f),
};
const uint8_t N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

void cargarDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC; cfg.version = CFG_VERSION;
  for (uint8_t i = 0; i < N_PARAMS; i++) {
    ParamDef d; memcpy_P(&d, &PARAMS[i], sizeof(d));
    *(float*)((uint8_t*)&cfg + d.ofs) = d.def;
  }
  cfg.fBlancoI = cfg.fBlancoD = 1.0f; cfg.fNegroI = cfg.fNegroD = 0.1f;
  cfg.rawMin = 300; cfg.rawMax = 1000; cfg.pendiente = 0.5f; cfg.thLinea = 90; cfg.pendLinea = -0.5f;
}

// ---------------------------------------------------------------- estado
enum Estado : uint8_t { REPOSO, MANUAL, CALIB, CALP, SIGUE, PERDIDO, ENSAYO, FALLA };
Estado estado = REPOSO;

Servo servo, pert;
int   ultUs = -1, ultUsP = -1;
float thOut = 90, thCmd = 90;     // brazo: ángulo enviado / pedido
float linOut = 0, linCmd = 0;     // línea (grados de brazo): enviada / pedida
float dt = 0.02f;

// medición
float rawI = 0, rawD = 0, cI = 0, cD = 0;
float bI = 0, bD = 0, sPos = 0, vis = 0, pMed = 0;
uint8_t ciclosFallaSensor = 0;
bool sensorMal = false;

// controlador
float ref = 0, eAct = 0, tP = 0, tI = 0, tD = 0, ePrev = 0, pPrev = 0;
bool  primerCiclo = true;
int8_t ladoUltimo = 1, dirBusqueda = 1;
uint8_t ciclosSinLinea = 0;
uint32_t tPerdido = 0;

// ensayo
enum TipoEns : uint8_t { E_PESC, E_RESC, E_SESC, E_PRAM, E_PSEN };
struct Ensayo { uint8_t tipo; bool disparado; float A, p2, dur, t, L0; };
Ensayo ens;

// calibración
enum FaseCal : uint8_t { CAL_IR, CAL_ESPERA, CAL_A, CAL_B, CAL_C, CAL_FIN };
FaseCal faseCal = CAL_IR;
uint32_t tFase = 0;
float calMinI, calMaxI, calMinD, calMaxD, rMin, rMax, thMinI, thMinD;
struct Pasada { float sPrev, xPrev; bool hayPrev; float cruce, fuerza, n, sx, sy, sxy, sxx; };
Pasada pasB, pasC;

// varios
bool telemetria = false, cambiosSinGuardar = false;
uint16_t seq = 0, atrasos = 0;
uint8_t divTel = 0, causaReset = 0;
uint32_t tUltCiclo = 0;

void procesarLinea(char* lin);
void enviarTelemetria();
void cambiarEstado(Estado e);

#if defined(__AVR__)
uint8_t mcusrCopia __attribute__((section(".noinit")));
void leerMCUSR(void) __attribute__((naked, used, section(".init3")));
void leerMCUSR(void) { mcusrCopia = MCUSR; MCUSR = 0; wdt_disable(); }
#else
uint8_t mcusrCopia = 0;
#endif

// ============================================================================
//  Utilidades
// ============================================================================
static inline float limitar(float x, float a, float b) { return x < a ? a : (x > b ? b : x); }
static inline float signo(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }

const __FlashStringHelper* nombreEstado(Estado e) {
  switch (e) {
    case REPOSO:  return F("REPOSO");
    case MANUAL:  return F("MANUAL");
    case CALIB:   return F("CALIB");
    case CALP:    return F("CALP");
    case SIGUE:   return F("SIGUE");
    case PERDIDO: return F("PERDIDO");
    case ENSAYO:  return F("ENSAYO");
    default:      return F("FALLA");
  }
}

uint16_t crc16(const uint8_t* p, uint16_t n) {
  uint16_t c = 0xFFFF;
  while (n--) { c ^= *p++; for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1); }
  return c;
}
bool cargarEEPROM() {
  Config c; EEPROM.get(0, c);
  if (c.magic != CFG_MAGIC || c.version != CFG_VERSION) return false;
  if (crc16((const uint8_t*)&c, offsetof(Config, crc)) != c.crc) return false;
  cfg = c; return true;
}
void guardarEEPROM() { cfg.crc = crc16((const uint8_t*)&cfg, offsetof(Config, crc)); EEPROM.put(0, cfg); }

void aplicarLeds() { analogWrite(PIN_LED_I, (int)cfg.ledi); analogWrite(PIN_LED_D, (int)cfg.ledd); }

void pc(float v, uint8_t d) { Serial.print(','); Serial.print(v, d); }
void ok(const __FlashStringHelper* t) { Serial.print(F("OK,")); Serial.println(t); }
void err(const __FlashStringHelper* t) { Serial.print(F("ERR,")); Serial.println(t); }

// ============================================================================
//  Medición
// ============================================================================
float leerADC(uint8_t pin) {
  analogRead(pin);
  uint16_t s = 0;
  for (uint8_t i = 0; i < 8; i++) s += analogRead(pin);
  return s / 8.0f;
}
static inline float conductancia(float raw) { raw = limitar(raw, 1.0f, 1022.0f); return raw / (1023.5f - raw); }

// Brillo corregido 0..1 (1 = blanco) según el modo de corrección
float brillo(float raw, bool izq) {
  uint8_t m = (uint8_t)cfg.corr;
  if (m == 1) {                                   // auto: conductancia normalizada por sensor
    float gB = izq ? cfg.fBlancoI : cfg.fBlancoD, gN = izq ? cfg.fNegroI : cfg.fNegroD;
    float d = gB - gN;
    return d < 1e-6f ? 0 : limitar((conductancia(raw) - gN) / d, 0, 1);
  }
  float rc = raw;
  if (m == 2) rc = izq ? raw * cfg.gi + cfg.oi : raw * cfg.gd + cfg.od;   // manual: ganancia y offset
  float d = cfg.rawMax - cfg.rawMin;                                     // 0 y 2: misma escala para ambos
  return d < 1 ? 0 : limitar((rc - cfg.rawMin) / d, 0, 1);
}

void medir(bool filtrar) {
  rawI = leerADC(PIN_LDR_I);
  rawD = leerADC(PIN_LDR_D);
  bool mal = (rawI < 2 || rawI > 1021 || rawD < 2 || rawD > 1021);
  if (mal) { if (ciclosFallaSensor < 255) ciclosFallaSensor++; } else ciclosFallaSensor = 0;
  sensorMal = (ciclosFallaSensor >= 10);
  cI = brillo(rawI, true);
  cD = brillo(rawD, false);
  float a = 1.0f;
  if (filtrar && cfg.tau > 0) a = cfg.per / (cfg.tau + cfg.per);
  bI += a * ((1 - cI) - bI);
  bD += a * ((1 - cD) - bD);
  vis = bI + bD;
  sPos = (vis > 0.05f) ? (bD - bI) / vis : 0.0f;
  pMed = sPos / cfg.pendiente;
}

// ============================================================================
//  Servos
// ============================================================================
int gradosAUs(float g) {
  int us = (int)(US_CENTRO + (g - 90.0f) * US_POR_GRADO + 0.5f);
  return us < 544 ? 544 : (us > 2400 ? 2400 : us);
}
void escribirServo(float th) {
  int us = gradosAUs(th + cfg.trim);
  if (us != ultUs) { servo.writeMicroseconds(us); ultUs = us; }
}
float anguloPert(float L) { return limitar(cfg.pcen + cfg.psig * cfg.prel * L, 0, 180); }
void escribirPert(float L) {
  int us = gradosAUs(anguloPert(L));
  if (us != ultUsP) { pert.writeMicroseconds(us); ultUsP = us; }
}
void moverBrazo() {
  thCmd = limitar(thCmd, cfg.amin, cfg.amax);
  if (cfg.lvel > 0.5f) { float paso = cfg.vmax * dt; thOut += limitar(thCmd - thOut, -paso, paso); }
  else thOut = thCmd;
  thOut = limitar(thOut, cfg.amin, cfg.amax);
  escribirServo(thOut);
}
void moverLinea() {
  linCmd = limitar(linCmd, -cfg.plmax, cfg.plmax);
  if (cfg.pvel > 0) { float paso = cfg.pvel * dt; linOut += limitar(linCmd - linOut, -paso, paso); }
  else linOut = linCmd;
  escribirPert(linOut);
}

// ============================================================================
//  Controlador PID
// ============================================================================
bool usaI() { uint8_t m = (uint8_t)cfg.modo; return m == 2 || m == 4; }
bool usaD() { uint8_t m = (uint8_t)cfg.modo; return m == 3 || m == 4; }

void reiniciarPID() {
  float e = ref - pMed;
  tP = cfg.kp * e; tD = 0;
  tI = usaI() ? limitar(thOut - cfg.u0 - tP, cfg.amin - cfg.u0, cfg.amax - cfg.u0) : 0;   // arranque sin salto
  ePrev = e; pPrev = pMed; primerCiclo = true;
}

void controlPID() {
  float e = ref - pMed;
  if (fabs(sPos) > 0.05f) ladoUltimo = (pMed < ref) ? 1 : -1;
  if (cfg.zm > 0) { if (fabs(e) <= cfg.zm) e = 0; else e -= signo(e) * cfg.zm; }
  eAct = e;

  tP = cfg.kp * e;

  if (usaD()) {
    float d = cfg.dmed > 0.5f ? -(pMed - pPrev) / dt : (e - ePrev) / dt;
    if (primerCiclo) d = 0;
    float a = cfg.kdf > 0 ? (cfg.per / (cfg.kdf + cfg.per)) : 1.0f;
    tD += a * (cfg.kd * d - tD);
  } else tD = 0;

  float u = cfg.u0 + tP + tI + tD;
  if (usaI()) {
    float kiEf = cfg.ki * ((fabs(sPos) > 0.9f) ? cfg.ksat : 1.0f);
    bool frena = false;
    if (cfg.aw > 0.5f) {
      frena = (u >= cfg.amax && e > 0) || (u <= cfg.amin && e < 0) ||
              (cfg.lvel > 0.5f && fabs(u - thOut) > cfg.vmax * dt * 1.5f && (u - thOut) * e > 0);
    }
    if (!frena) tI += kiEf * e * dt;
    if (cfg.aw > 0.5f) tI = limitar(tI, cfg.amin - cfg.u0, cfg.amax - cfg.u0);
    else tI = limitar(tI, -400, 400);
    u = cfg.u0 + tP + tI + tD;
  } else tI = 0;

  thCmd = limitar(u, cfg.amin, cfg.amax);
  ePrev = e; pPrev = pMed; primerCiclo = false;
}

void iniciarSeguimiento() {
  ciclosSinLinea = 0;
  if (vis >= cfg.vison) { reiniciarPID(); cambiarEstado(SIGUE); }
  else {
    tPerdido = millis() - (uint32_t)cfg.thold;
    dirBusqueda = (cfg.thLinea >= thOut) ? 1 : -1;
    thCmd = thOut;
    cambiarEstado(PERDIDO);
  }
}

// ============================================================================
//  Ensayos
// ============================================================================
void iniciarEnsayo(uint8_t tipo, float A, float p2) {
  memset(&ens, 0, sizeof(ens));
  ens.tipo = tipo; ens.A = A; ens.p2 = p2;
  ens.dur = (tipo == E_PSEN) ? (5.0f / p2) : cfg.tdur;
  if (tipo == E_SESC) thCmd = thOut;          // lazo abierto: el brazo queda donde está
  cambiarEstado(ENSAYO);
  if (fabs(A) * fabs(cfg.pendiente) > 0.9f && tipo != E_SESC) Serial.println(F("I,ens,amplitud mayor que la zona lineal"));
  // E,inicio,tipo,A,p2,pre,duración,modo,kp,ki,kd,flags(dmed aw lvel corr),zm,per
  Serial.print(F("E,inicio,")); Serial.print(tipo); pc(A, 2); pc(p2, 3); pc(cfg.tpre, 2); pc(ens.dur, 2);
  pc(cfg.modo, 0); pc(cfg.kp, 3); pc(cfg.ki, 3); pc(cfg.kd, 4);
  pc(cfg.dmed * 1000 + cfg.aw * 100 + cfg.lvel * 10 + cfg.corr, 0); pc(cfg.zm, 2); pc(cfg.per, 0); Serial.println();
}

// Las métricas (subida, sobrepaso, establecimiento, IAE, ISE, ganancia y fase) las calcula
// el panel con la telemetría, que durante el ensayo se envía en cada ciclo.
void finEnsayo(bool abortado) {
  if (abortado) err(F("ens,abortado"));
  else { Serial.print(F("E,fin,")); Serial.println(seq); }
  ref = 0;                                   // la referencia vuelve a 0 (solo la cambia "resc")
  if (ens.tipo == E_SESC) { thCmd = thOut; cambiarEstado(REPOSO); }
  else if (!abortado) { reiniciarPID(); cambiarEstado(SIGUE); }
}

void pasoEnsayo() {
  Ensayo& E = ens;
  E.t += dt;
  bool lazo = (E.tipo != E_SESC);
  if (lazo && vis < cfg.visoff) {
    if (++ciclosSinLinea >= 5) { finEnsayo(true); tPerdido = millis(); dirBusqueda = ladoUltimo; thCmd = thOut; cambiarEstado(PERDIDO); return; }
  } else ciclosSinLinea = 0;

  float tt = E.t - cfg.tpre;                       // tiempo desde el disparo
  if (!E.disparado && tt >= 0) {
    E.disparado = true; E.L0 = linOut;
    // E,disparo,seq,L0,ref,th,p   (seq de la telemetría de este mismo ciclo)
    Serial.print(F("E,disparo,")); Serial.print(seq); pc(linOut, 2); pc(ref, 2); pc(thOut, 2); pc(pMed, 2); Serial.println();
    switch (E.tipo) {
      case E_PESC: linCmd = linOut = limitar(E.L0 + E.A, -cfg.plmax, cfg.plmax); escribirPert(linOut); break;
      case E_RESC: ref += E.A; break;
      case E_SESC: thCmd = thOut = limitar(thOut + E.A, cfg.amin, cfg.amax); escribirServo(thOut); break;
      default: break;
    }
  }
  if (E.disparado && E.tipo >= E_PRAM) {
    float d = (E.tipo == E_PRAM) ? limitar(E.p2 * tt, 0, fabs(E.A)) * signo(E.A) : E.A * sin(6.2832f * E.p2 * tt);
    linCmd = linOut = limitar(E.L0 + d, -cfg.plmax, cfg.plmax); escribirPert(linOut);
  }
  if (lazo) { controlPID(); moverBrazo(); }
  if (tt >= E.dur) finEnsayo(false);
}

// ============================================================================
//  Calibraciones
// ============================================================================
void reiniciarPasada(Pasada& p) { memset(&p, 0, sizeof(p)); p.cruce = NAN; }
void acumular(Pasada& p, float x, float s, float v) {
  if (v >= cfg.vison && fabs(s) < 0.6f) { p.n += 1; p.sx += x; p.sy += s; p.sxy += x * s; p.sxx += x * x; }
  if (p.hayPrev && v >= cfg.vison && ((p.sPrev < 0 && s >= 0) || (p.sPrev > 0 && s <= 0))) {
    float f = fabs(s - p.sPrev);
    if (f > p.fuerza) { p.fuerza = f; p.cruce = p.xPrev + (x - p.xPrev) * (0 - p.sPrev) / (s - p.sPrev); }
  }
  p.sPrev = s; p.xPrev = x; p.hayPrev = true;
}
float pendientePasada(const Pasada& p) {
  float den = p.n * p.sxx - p.sx * p.sx;
  if (p.n < 4 || fabs(den) < 1e-6f) return NAN;
  return (p.n * p.sxy - p.sx * p.sy) / den;
}
void falloCal(const __FlashStringHelper* m) {
  Serial.print(F("ERR,cal,")); Serial.println(m);
  thCmd = thOut; linCmd = linOut; cambiarEstado(REPOSO);
}
void puntoCal(uint8_t pas, float x, float s) {
  Serial.print(F("CS,")); Serial.print(pas); pc(x, 2); pc(s, 3); pc(rawI, 0); pc(rawD, 0); Serial.println();
}

// --- cal: barrido del brazo (A extremos y diagnóstico, B y C centro y pendiente)
void iniciarCal() {
  faseCal = CAL_IR; thCmd = cfg.amin;
  calMinI = calMinD = rMin = 1e9f; calMaxI = calMaxD = rMax = -1e9f;
  reiniciarPasada(pasB); reiniciarPasada(pasC);
  cambiarEstado(CALIB);
  Serial.println(F("C,INICIO,brazo"));
}
void pasoCal() {
  float gI = conductancia(rawI), gD = conductancia(rawD);
  switch (faseCal) {
    case CAL_IR:
      thCmd = cfg.amin;
      if (fabs(thOut - cfg.amin) < 0.05f) { faseCal = CAL_ESPERA; tFase = millis(); }
      break;
    case CAL_ESPERA:
      if (millis() - tFase >= 500) { faseCal = CAL_A; Serial.println(F("C,PASADA,A")); }
      break;
    case CAL_A:
      if (gI < calMinI) { calMinI = gI; thMinI = thOut; }
      if (gD < calMinD) { calMinD = gD; thMinD = thOut; }
      if (gI > calMaxI) calMaxI = gI;
      if (gD > calMaxD) calMaxD = gD;
      rMin = min(rMin, min(rawI, rawD)); rMax = max(rMax, max(rawI, rawD));
      if (seq % 2 == 0) puntoCal(0, thOut, 0);
      thCmd = thOut + cfg.vcal * dt;
      if (thCmd >= cfg.amax) {
        thCmd = cfg.amax;
        // diagnóstico: contraste, saturación y desfase entre los LDR
        float rawBI = 1023.5f * calMaxI / (1 + calMaxI), rawBD = 1023.5f * calMaxD / (1 + calMaxD);
        // C,DIAG,contrasteI,contrasteD,rawBlancoI,rawBlancoD,ángulo mín I,ángulo mín D,desfase
        Serial.print(F("C,DIAG")); pc(calMaxI / calMinI, 2); pc(calMaxD / calMinD, 2); pc(rawBI, 0); pc(rawBD, 0);
        pc(thMinI, 1); pc(thMinD, 1); pc(thMinD - thMinI, 1); Serial.println();
        if (calMaxI < 1.3f * calMinI || calMaxD < 1.3f * calMinD) { falloCal(F("contraste insuficiente")); return; }
        if (fabs(thMinD - thMinI) < 3.0f) { falloCal(F("desfase LDR chico: la cinta debe apuntar al eje")); return; }
        cfg.fBlancoI = calMaxI; cfg.fNegroI = calMinI; cfg.fBlancoD = calMaxD; cfg.fNegroD = calMinD;
        cfg.rawMin = rMin; cfg.rawMax = rMax;
        faseCal = CAL_B; Serial.println(F("C,PASADA,B"));
      }
      break;
    case CAL_B:
    case CAL_C: {
      Pasada& p = (faseCal == CAL_B) ? pasB : pasC;
      acumular(p, thOut - 90.0f, sPos, vis);
      if (seq % 2 == 0) puntoCal(faseCal == CAL_B ? 1 : 2, thOut, sPos);
      if (faseCal == CAL_B) {
        thCmd = thOut - cfg.vcal * dt;
        if (thCmd <= cfg.amin) { thCmd = cfg.amin; faseCal = CAL_C; Serial.println(F("C,PASADA,C")); }
      } else {
        thCmd = thOut + cfg.vcal * dt;
        if (thCmd >= cfg.amax) { thCmd = cfg.amax; faseCal = CAL_FIN; }
      }
      break;
    }
    case CAL_FIN: {
      float kB = pendientePasada(pasB), kC = pendientePasada(pasC);
      if (isnan(pasB.cruce) || isnan(pasC.cruce)) { falloCal(F("centro no encontrado")); return; }
      if (isnan(kB) || isnan(kC) || kB * kC <= 0) { falloCal(F("pendiente inconsistente")); return; }
      float k = 0.5f * (kB + kC);
      if (fabs(k) < 0.05f || fabs(k) > 5.0f) { falloCal(F("pendiente fuera de rango")); return; }
      if (fabs(pasB.cruce - pasC.cruce) > 8.0f) { falloCal(F("cruces distintos")); return; }
      cfg.pendiente = k; cfg.thLinea = 90.0f + 0.5f * (pasB.cruce + pasC.cruce);
      cfg.calOk = 1; cfg.calModo = (uint8_t)cfg.corr; cambiosSinGuardar = true;
      Serial.print(F("C,OK")); pc(cfg.thLinea, 2); pc(k, 4); pc(0.5f * fabs(pasB.cruce - pasC.cruce), 2); Serial.println();
      thCmd = cfg.thLinea; bI = bD = 0; medir(false);
      cambiarEstado(MANUAL);
      break;
    }
  }
}

// --- calp: el brazo queda quieto en 90° y el perturbador barre la línea
void iniciarCalp() {
  if (!cfg.calOk) { err(F("calp,falta cal")); return; }
  faseCal = CAL_IR; thCmd = 90; linCmd = -cfg.plmax;
  reiniciarPasada(pasB); reiniciarPasada(pasC);
  cambiarEstado(CALP);
  Serial.println(F("C,INICIO,perturbador"));
}
void pasoCalp() {
  switch (faseCal) {
    case CAL_IR:
      thCmd = 90; linCmd = -cfg.plmax;
      if (fabs(thOut - 90) < 0.05f && fabs(linOut + cfg.plmax) < 0.05f) { faseCal = CAL_ESPERA; tFase = millis(); }
      break;
    case CAL_ESPERA:
      if (millis() - tFase >= 700) { faseCal = CAL_B; Serial.println(F("C,PASADA,B")); }
      break;
    case CAL_B:
    case CAL_C: {
      Pasada& p = (faseCal == CAL_B) ? pasB : pasC;
      acumular(p, linOut, sPos, vis);
      if (seq % 2 == 0) puntoCal(faseCal == CAL_B ? 3 : 4, linOut, sPos);
      float paso = cfg.vcal * 0.5f * dt;
      if (faseCal == CAL_B) { linCmd = linOut + paso; if (linCmd >= cfg.plmax) { linCmd = cfg.plmax; faseCal = CAL_C; Serial.println(F("C,PASADA,C")); } }
      else { linCmd = linOut - paso; if (linCmd <= -cfg.plmax) { linCmd = -cfg.plmax; faseCal = CAL_FIN; } }
      linOut = linCmd;          // movimiento continuo y lento (sin el límite pvel)
      break;
    }
    default: {
      float kB = pendientePasada(pasB), kC = pendientePasada(pasC);
      if (isnan(pasB.cruce) || isnan(pasC.cruce) || isnan(kB) || isnan(kC) || kB * kC <= 0) { falloCal(F("calp: linea no encontrada")); return; }
      float kL = 0.5f * (kB + kC);
      float Lc = 0.5f * (pasB.cruce + pasC.cruce);
      // esperado: kL = -pendiente (mover la línea +1° equivale a mover el brazo -1°)
      float ratio = -kL / cfg.pendiente;
      float psigViejo = cfg.psig, prelViejo = cfg.prel;
      float pcenNuevo = anguloPert(Lc);
      if (ratio < 0) { cfg.psig = -cfg.psig; ratio = -ratio; }
      if (ratio < 0.5f || ratio > 2.0f) { falloCal(F("calp: relacion fuera de rango")); return; }
      cfg.prel = prelViejo / ratio;
      cfg.pcen = pcenNuevo;
      cfg.pendLinea = kL; cfg.calpOk = 1; cambiosSinGuardar = true;
      linOut = linCmd = 0; escribirPert(0);
      // CP,OK,pcen,prel,psig,relación medida,sentido invertido,retardo
      Serial.print(F("CP,OK")); pc(cfg.pcen, 2); pc(cfg.prel, 3); pc(cfg.psig, 0); pc(ratio, 3);
      pc(psigViejo != cfg.psig, 0); pc(0.5f * fabs(pasB.cruce - pasC.cruce), 2); Serial.println();
      thCmd = thOut; cambiarEstado(REPOSO);
      break;
    }
  }
}

// ============================================================================
//  Ciclo
// ============================================================================
void cambiarEstado(Estado e) {
  if (e == estado) return;
  estado = e; Serial.print(F("S,")); Serial.println(nombreEstado(e));
}

void ciclo() {
  medir(estado != CALIB && estado != CALP);
  if (sensorMal && (estado == SIGUE || estado == PERDIDO || estado == CALIB || estado == CALP || estado == ENSAYO)) {
    err(F("sensor,LDR fuera de rango"));
    if (estado == ENSAYO) finEnsayo(true);
    thCmd = thOut; linCmd = linOut; cambiarEstado(FALLA);
  }
  switch (estado) {
    case REPOSO: case FALLA: eAct = 0; tP = tI = tD = 0; break;
    case MANUAL: if (fabs(thOut - thCmd) < 0.05f) cambiarEstado(REPOSO); break;
    case SIGUE:
      if (vis < cfg.visoff) {
        if (++ciclosSinLinea >= 5) { tPerdido = millis(); dirBusqueda = ladoUltimo; thCmd = thOut; cambiarEstado(PERDIDO); }
      } else { ciclosSinLinea = 0; controlPID(); }
      break;
    case PERDIDO:
      eAct = 0;
      if (vis >= cfg.vison) { iniciarSeguimiento(); break; }
      if (cfg.busq > 0.5f && millis() - tPerdido >= (uint32_t)cfg.thold) {
        thCmd = thOut + dirBusqueda * cfg.vbusq * dt;
        if (thCmd >= cfg.amax) { thCmd = cfg.amax; dirBusqueda = -1; }
        if (thCmd <= cfg.amin) { thCmd = cfg.amin; dirBusqueda = 1; }
      } else thCmd = thOut;
      break;
    case ENSAYO: pasoEnsayo(); break;
    case CALIB: pasoCal(); break;
    case CALP: pasoCalp(); break;
  }
  if (estado != ENSAYO) {
    moverBrazo();
    if (estado == CALP && faseCal >= CAL_B) escribirPert(linOut);   // barrido continuo de la calibración
    else moverLinea();
  }

  if (telemetria) {
    uint8_t minDiv = (uint8_t)((20 + (uint8_t)cfg.per - 1) / (uint8_t)cfg.per);     // máx. ~50 líneas/s
    uint8_t div = (estado == ENSAYO) ? minDiv : max(minDiv, (uint8_t)cfg.teldiv);
    if (++divTel >= div) { divTel = 0; enviarTelemetria(); }
  }
}

// ============================================================================
//  Salida de datos
// ============================================================================
void enviarTelemetria() {
  // T,seq,ms,estado,rawI,rawD,cI,cD,s,p,ref,e,P,I,D,u,th,L
  Serial.print(F("T,")); Serial.print(seq); Serial.print(','); Serial.print(millis()); Serial.print(',');
  Serial.print(nombreEstado(estado));
  pc(rawI, 0); pc(rawD, 0); pc(cI, 3); pc(cD, 3); pc(sPos, 3); pc(pMed, 2); pc(ref, 2); pc(eAct, 2);
  pc(tP, 2); pc(tI, 2); pc(tD, 2); pc(thCmd, 2); pc(thOut, 2); pc(linOut, 2); Serial.println();
}
void enviarParametro(uint8_t i) {
  ParamDef d; memcpy_P(&d, &PARAMS[i], sizeof(d));
  Serial.print(F("P,")); Serial.print(d.nombre); pc(*(float*)((uint8_t*)&cfg + d.ofs), 4);
  pc(d.minv, 3); pc(d.maxv, 3); pc(d.tipo, 0); Serial.println();
}
void enviarCalibracion() {
  // K,calOk,calModo,fBI,fNI,fBD,fND,rawMin,rawMax,pendiente,thLinea,calpOk,pendLinea
  Serial.print(F("K,")); Serial.print(cfg.calOk); pc(cfg.calModo, 0); pc(cfg.fBlancoI, 4); pc(cfg.fNegroI, 4);
  pc(cfg.fBlancoD, 4); pc(cfg.fNegroD, 4); pc(cfg.rawMin, 0); pc(cfg.rawMax, 0); pc(cfg.pendiente, 4);
  pc(cfg.thLinea, 2); pc(cfg.calpOk, 0); pc(cfg.pendLinea, 4); Serial.println();
}
void enviarStatus() {
  // ST,estado,calOk,sinGuardar,atrasos,th,tel,L,calpOk
  Serial.print(F("ST,")); Serial.print(nombreEstado(estado)); pc(cfg.calOk, 0); pc(cambiosSinGuardar, 0);
  pc(atrasos, 0); pc(thOut, 2); pc(telemetria, 0); pc(linOut, 2); pc(cfg.calpOk, 0); Serial.println();
}
void enviarAyuda() {
  Serial.println(F("I,get set run stop cal calp goto lin lcero ens save load defaults tel status"));
}

// ============================================================================
//  Órdenes
// ============================================================================
char bufLinea[40];
uint8_t lenLinea = 0;
bool lineaDescartada = false;

void leerSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineaDescartada) { err(F("linea demasiado larga")); lineaDescartada = false; }
      else if (lenLinea > 0) { bufLinea[lenLinea] = 0; procesarLinea(bufLinea); }
      lenLinea = 0;
    } else if (!lineaDescartada) {
      if (lenLinea < sizeof(bufLinea) - 1) bufLinea[lenLinea++] = c;
      else { lineaDescartada = true; lenLinea = 0; }
    }
  }
}
// número con punto o coma decimal (sin strtod: ahorra memoria de programa)
bool leerNumero(const char* t, float& v) {
  if (!t) return false;
  bool neg = (*t == '-');
  if (*t == '-' || *t == '+') t++;
  float r = 0, div = 1; bool dig = false, punto = false;
  for (; *t; t++) {
    if (*t >= '0' && *t <= '9') { dig = true; if (punto) { div *= 10; r += (*t - '0') / div; } else r = r * 10 + (*t - '0'); }
    else if ((*t == '.' || *t == ',') && !punto) punto = true;
    else return false;
  }
  if (!dig) return false;
  v = neg ? -r : r; return true;
}

void detener() {
  if (estado == ENSAYO) finEnsayo(true);
  thCmd = thOut; linCmd = linOut; ref = 0; cambiarEstado(REPOSO);
}

bool aplicarParametro(const char* nombre, float v) {
  for (uint8_t i = 0; i < N_PARAMS; i++) {
    ParamDef d; memcpy_P(&d, &PARAMS[i], sizeof(d));
    if (strcmp(nombre, d.nombre)) continue;
    if (d.tipo) v = (float)(long)(v + (v >= 0 ? 0.5f : -0.5f));
    if (v < d.minv || v > d.maxv || (!strcmp(d.nombre, "psig") && v == 0)) {
      Serial.print(F("ERR,set,")); Serial.print(d.nombre); Serial.println(F(",fuera de rango")); enviarParametro(i); return true;
    }
    float* dst = (float*)((uint8_t*)&cfg + d.ofs);
    float viejo = *dst; *dst = v;
    if (cfg.visoff >= cfg.vison) { *dst = viejo; err(F("set,visoff >= vison")); enviarParametro(i); return true; }
    if (!strcmp(d.nombre, "ledi") || !strcmp(d.nombre, "ledd")) aplicarLeds();
    if (!strcmp(d.nombre, "trim")) escribirServo(thOut);
    if (!strcmp(d.nombre, "per")) dt = cfg.per / 1000.0f;
    if (!strcmp(d.nombre, "pcen") || !strcmp(d.nombre, "prel") || !strcmp(d.nombre, "psig")) escribirPert(linOut);
    if (!strcmp(d.nombre, "modo") || !strcmp(d.nombre, "u0")) { if (estado == SIGUE) reiniciarPID(); }
    if (!strcmp(d.nombre, "corr")) { bI = bD = 0; }
    cambiosSinGuardar = true;
    enviarParametro(i);
    return true;
  }
  return false;
}

void procesarLinea(char* lin) {
  for (char* p = lin; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;
  char* cmd = strtok(lin, " \t");
  if (!cmd) return;
  char* a1 = strtok(NULL, " \t");
  char* a2 = strtok(NULL, " \t");
  char* a3 = strtok(NULL, " \t");
  float v, w;

  if (!strcmp(cmd, "stop")) { detener(); ok(F("stop")); return; }
  if (!strcmp(cmd, "get")) {
    for (uint8_t i = 0; i < N_PARAMS; i++) enviarParametro(i);
    enviarCalibracion(); enviarStatus(); Serial.println(F("END")); return;
  }
  if (!strcmp(cmd, "status")) { enviarStatus(); return; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "ayuda")) { enviarAyuda(); return; }
  if (!strcmp(cmd, "tel")) {
    if (!leerNumero(a1, v)) { err(F("tel,usar tel 1 o tel 0")); return; }
    telemetria = (v != 0); divTel = 0; Serial.print(F("OK,tel,")); Serial.println(telemetria ? 1 : 0); return;
  }
  if (!strcmp(cmd, "set")) {
    if (!a1 || !leerNumero(a2, v)) { err(F("set,usar set <nombre> <valor>")); return; }
    // en vivo durante el seguimiento y los ensayos: solo parámetros del controlador y la telemetría
    if (estado == CALIB || estado == CALP) { err(F("set,ocupado (calibrando)")); return; }
    if (!aplicarParametro(a1, v)) { Serial.print(F("ERR,set,parametro desconocido: ")); Serial.println(a1); }
    return;
  }
  if (!strcmp(cmd, "lin")) {
    if (!leerNumero(a1, v)) { err(F("lin,usar lin <grados>")); return; }
    if (estado == CALIB || estado == CALP || estado == ENSAYO) { err(F("lin,ocupado")); return; }
    linCmd = limitar(v, -cfg.plmax, cfg.plmax); Serial.print(F("OK,lin,")); Serial.println(linCmd, 2); return;
  }

  if (estado == CALIB || estado == CALP || estado == ENSAYO) { Serial.print(F("ERR,")); Serial.print(cmd); Serial.println(F(",ocupado (usar stop)")); return; }

  if (!strcmp(cmd, "run")) {
    if (!cfg.calOk) { err(F("run,sin calibrar (usar cal)")); return; }
    if (sensorMal) { err(F("run,sensor fuera de rango")); return; }
    if (estado == SIGUE) { ok(F("run")); return; }
    ref = 0; iniciarSeguimiento(); ok(F("run")); return;
  }
  if (!strcmp(cmd, "cal")) { iniciarCal(); return; }
  if (!strcmp(cmd, "calp")) { iniciarCalp(); return; }
  if (!strcmp(cmd, "goto")) {
    if (!leerNumero(a1, v)) { err(F("goto,usar goto <grados>")); return; }
    thCmd = limitar(v, cfg.amin, cfg.amax); cambiarEstado(MANUAL);
    Serial.print(F("OK,goto,")); Serial.println(thCmd, 2); return;
  }
  if (!strcmp(cmd, "lcero")) {
    cfg.pcen = anguloPert(linOut); linOut = linCmd = 0; escribirPert(0); cambiosSinGuardar = true;
    Serial.print(F("OK,lcero,")); Serial.println(cfg.pcen, 2); return;
  }
  if (!strcmp(cmd, "ens")) {
    if (!a1 || !leerNumero(a2, v)) { err(F("ens,usar ens <tipo> <A> [p2]")); return; }
    uint8_t t;
    if (!strcmp(a1, "pesc")) t = E_PESC; else if (!strcmp(a1, "resc")) t = E_RESC; else if (!strcmp(a1, "sesc")) t = E_SESC;
    else if (!strcmp(a1, "pram")) t = E_PRAM; else if (!strcmp(a1, "psen")) t = E_PSEN;
    else { err(F("ens,tipo desconocido")); return; }
    if (fabs(v) < 0.2f || fabs(v) > 20) { err(F("ens,amplitud 0,2..20")); return; }
    w = 0;
    if (t == E_PRAM || t == E_PSEN) {
      if (!leerNumero(a3, w) || w <= 0) { err(F("ens,falta p2")); return; }
      if (t == E_PSEN && (w < 0.05f || w > 3)) { err(F("ens,f 0,05..3 Hz")); return; }
      if (t == E_PRAM && w > 100) { err(F("ens,vel max 100")); return; }
    }
    if (t == E_SESC) {
      if (estado != REPOSO && estado != MANUAL) { err(F("ens,sesc requiere stop")); return; }
    } else {
      if (estado != SIGUE) { err(F("ens,requiere run")); return; }
      if (t != E_RESC && fabs(linOut + v) > cfg.plmax + 0.01f) { err(F("ens,fuera de plmax")); return; }
    }
    iniciarEnsayo(t, v, w); return;
  }
  if (!strcmp(cmd, "save")) { guardarEEPROM(); cambiosSinGuardar = false; ok(F("save")); return; }
  if (!strcmp(cmd, "load")) {
    if (cargarEEPROM()) { aplicarLeds(); dt = cfg.per / 1000.0f; cambiosSinGuardar = false; detener(); ok(F("load")); }
    else err(F("load,EEPROM sin datos validos"));
    return;
  }
  if (!strcmp(cmd, "defaults")) {
    cargarDefaults(); aplicarLeds(); dt = cfg.per / 1000.0f; cambiosSinGuardar = true; detener();
    ok(F("defaults")); return;
  }
  Serial.print(F("ERR,orden desconocida: ")); Serial.println(cmd);
}

// ============================================================================
//  setup / loop
// ============================================================================
void setup() {
  causaReset = mcusrCopia;
  Serial.begin(115200);
  bool desdeEEPROM = cargarEEPROM();
  if (!desdeEEPROM) cargarDefaults();
  dt = cfg.per / 1000.0f;
  pinMode(PIN_LED_I, OUTPUT); pinMode(PIN_LED_D, OUTPUT);
  aplicarLeds();

  thOut = thCmd = 90; linOut = linCmd = 0;
  escribirServo(thOut); escribirPert(linOut);              // el pulso se fija antes de attach
  servo.attach(PIN_SERVO, 544, 2400);
  pert.attach(PIN_PERT, 544, 2400);

  delay(300);
  medir(false);
  Serial.print(F("BOOT,banco_pid," FW_VERSION ","));
  if (causaReset & 0x08)      Serial.print(F("watchdog"));
  else if (causaReset & 0x04) Serial.print(F("brown-out"));
  else if (causaReset & 0x02) Serial.print(F("reset/puerto"));
  else if (causaReset & 0x01) Serial.print(F("encendido"));
  else                        Serial.print(F("desconocida"));
  Serial.print(','); Serial.println(desdeEEPROM ? F("config EEPROM") : F("config de fabrica"));
  enviarAyuda();
  enviarStatus();
#if USAR_WATCHDOG && defined(__AVR__)
  wdt_enable(WDTO_500MS);
#endif
  tUltCiclo = millis();
}

void loop() {
  leerSerie();
  uint32_t ahora = millis();
  uint16_t per = (uint16_t)cfg.per;
  if (ahora - tUltCiclo >= per) {
    tUltCiclo += per;
    if (ahora - tUltCiclo > 5u * per) { tUltCiclo = ahora; atrasos++; }
    seq++;
    ciclo();
  }
#if USAR_WATCHDOG && defined(__AVR__)
  wdt_reset();
#endif
}
