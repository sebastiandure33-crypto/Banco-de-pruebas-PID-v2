/*
  ============================================================================
  SEGUIDOR DE LÍNEA ESTÁTICO v2  —  Arduino Nano (ATmega328P) + servo SG90
  ============================================================================
  Dos LDR miran el piso desde el cabezal del brazo. El servo gira el brazo
  (60°–120°) para que la línea negra quede centrada entre los dos sensores.

  CONEXIONES
    LDR izquierdo : 5V — LDR — A0 — R — GND
    LDR derecho   : 5V — LDR — A1 — R — GND      (R ≈ 4,7k a 10k, ver LEEME)
    Servo (señal) : D9         (alimentación aparte, masa común con el Nano)
    LED izquierdo : D5 — resistencia (220–330 Ω) — LED — GND
    LED derecho   : D6 — resistencia (220–330 Ω) — LED — GND
    (D9/D10 pierden el PWM por la librería Servo: por eso los LED van en D5/D6)

  CÓMO FUNCIONA
    1. Medición: cada lectura se pasa a CONDUCTANCIA del LDR (proporcional a la
       luz) y se normaliza entre el negro y el blanco de ESE sensor. Así dos LDR
       con sensibilidad muy distinta quedan comparables, sin factor de balance.
    2. Posición de la línea: s = (negroDer − negroIzq) / (negroDer + negroIzq),
       entre −1 y +1. s = 0 → línea centrada.
    3. La calibración (comando "cal") barre el brazo sobre la línea en ambos
       sentidos y mide la pendiente de s por grado. Con eso el error se expresa
       en GRADOS: e = −s / pendiente.
    4. Control PI posicional (UNA sola integración):
           ánguloObjetivo = integral + Kp·e        integral += Ki·e·dt
       El servo es un actuador de posición: el integrador acumula el ángulo
       donde está la línea y Kp agrega respuesta inmediata.
    5. Línea perdida: no se inventa el error. Se mantiene la posición "thold" ms
       y después busca despacio hacia el lado donde se la vio por última vez.

  PROTOCOLO SERIE (115200 baud, una orden por línea terminada en Enter)
    ?                 ayuda
    get               lista todos los parámetros y la calibración
    set <nom> <val>   cambia un parámetro (responde con el valor aplicado)
    run | stop        arranca / detiene el control
    cal               calibración por barrido (la línea debe estar bajo el brazo)
    goto <grados>     mueve el brazo a un ángulo (detiene el control)
    esc <grados>      ensayo de escalón: salto brusco y registro de 2 s
    save | load       guarda / recarga la configuración en EEPROM
    defaults          valores de fábrica (pierde la calibración en RAM)
    tel 1 | tel 0     telemetría continua on / off
    status            estado actual
  Respuestas: OK,…  ERR,…  P,… (parámetro)  S,… (cambio de estado)
              T,… (telemetría)  C,…/CS,… (calibración)  BOOT,… (arranque)
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

#define FW_VERSION "2.0"

// Watchdog: reinicia el Nano si el programa se cuelga. APAGADO por defecto:
// muchos Nano baratos traen el bootloader viejo, que no desactiva el watchdog
// y queda reiniciándose en bucle tras un reset por watchdog. Activalo (1) solo
// si tu placa usa Optiboot (en el IDE: "ATmega328P", no "Old Bootloader").
#define USAR_WATCHDOG 0

// ---------------------------------------------------------------- pines
const uint8_t PIN_LDR_I = A0;
const uint8_t PIN_LDR_D = A1;
const uint8_t PIN_SERVO = 9;
const uint8_t PIN_LED_I = 5;
const uint8_t PIN_LED_D = 6;

// ---------------------------------------------------------------- tiempo
const uint16_t PERIODO_MS = 20;            // lazo a 50 Hz (= refresco del servo)
const float    DT = PERIODO_MS / 1000.0f;

// ---------------------------------------------------------------- servo SG90
const float US_CENTRO  = 1472.0f;          // 90° (mismo mapa que Servo.write)
const float US_POR_GRADO = (2400.0f - 544.0f) / 180.0f;

// ---------------------------------------------------------------- configuración persistente
struct Config {
  uint16_t magic;
  uint8_t  version;
  // control
  float kp, ki, ksat, zm;
  // movimiento
  float vmax, vbusq, vcal, thold, amin, amax, trim;
  // sensores
  float tau, vison, visoff, ledi, ledd, teldiv;
  // calibración (la escribe "cal")
  uint8_t calOk;
  float fBlancoI, fNegroI, fBlancoD, fNegroD;   // conductancia relativa
  float pendiente;                              // s por grado (con signo)
  float thLinea;                                // ángulo de la línea al calibrar
  uint16_t crc;
};
const uint16_t CFG_MAGIC = 0x5347;   // 'SG'
const uint8_t  CFG_VERSION = 2;

Config cfg;

void cargarDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = CFG_MAGIC; cfg.version = CFG_VERSION;
  cfg.kp = 0.40f;  cfg.ki = 6.0f;  cfg.ksat = 3.0f; cfg.zm = 0.20f;
  cfg.vmax = 150;  cfg.vbusq = 20; cfg.vcal = 8;    cfg.thold = 300;
  cfg.amin = 60;   cfg.amax = 120; cfg.trim = 0;
  cfg.tau = 40;    cfg.vison = 0.35f; cfg.visoff = 0.20f;
  cfg.ledi = 255;  cfg.ledd = 255;  cfg.teldiv = 2;
  cfg.calOk = 0;
  cfg.fBlancoI = cfg.fBlancoD = 1.0f; cfg.fNegroI = cfg.fNegroD = 0.1f;
  cfg.pendiente = 0.5f; cfg.thLinea = 90;
}

// Tabla de parámetros ajustables: nombre, posición en Config, mínimo, máximo.
struct ParamDef { char nombre[8]; uint8_t ofs; float minv, maxv; };
const ParamDef PARAMS[] PROGMEM = {
  {"kp",     offsetof(Config, kp),     0.0f,  1.5f},
  {"ki",     offsetof(Config, ki),     0.0f,  20.0f},
  {"ksat",   offsetof(Config, ksat),   1.0f,  10.0f},
  {"zm",     offsetof(Config, zm),     0.0f,  3.0f},
  {"vmax",   offsetof(Config, vmax),   10.0f, 600.0f},
  {"vbusq",  offsetof(Config, vbusq),  2.0f,  200.0f},
  {"vcal",   offsetof(Config, vcal),   2.0f,  30.0f},
  {"thold",  offsetof(Config, thold),  0.0f,  5000.0f},
  {"amin",   offsetof(Config, amin),   30.0f, 89.0f},
  {"amax",   offsetof(Config, amax),   91.0f, 150.0f},
  {"trim",   offsetof(Config, trim),  -30.0f, 30.0f},
  {"tau",    offsetof(Config, tau),    0.0f,  500.0f},
  {"vison",  offsetof(Config, vison),  0.05f, 1.5f},
  {"visoff", offsetof(Config, visoff), 0.0f,  1.2f},
  {"ledi",   offsetof(Config, ledi),   0.0f,  255.0f},
  {"ledd",   offsetof(Config, ledd),   0.0f,  255.0f},
  {"teldiv", offsetof(Config, teldiv), 1.0f,  50.0f},
};
const uint8_t N_PARAMS = sizeof(PARAMS) / sizeof(PARAMS[0]);

// ---------------------------------------------------------------- estado
enum Estado : uint8_t { REPOSO, MANUAL, CALIB, SIGUE, PERDIDO, ESCALON, FALLA };
Estado estado = REPOSO;

Servo servo;
int   ultimoUs = -1;
float thOut = 90;        // ángulo lógico enviado al servo (tras limitar velocidad)
float thCmd = 90;        // ángulo pedido
float thI = 90;          // integrador del PI (ángulo acumulado)
float eGrados = 0;       // error actual en grados

// medición
float rawI = 0, rawD = 0;            // ADC promedio (0..1023)
float bI = 0, bD = 0;                // "negrura" filtrada 0..1 (1 = negro)
float sPos = 0;                      // posición normalizada de la línea (−1..1)
float vis = 0;                       // bI + bD: cuánta línea se ve
uint8_t ciclosFallaSensor = 0;
bool  sensorMal = false;

// seguimiento / pérdida
int8_t   ladoUltimo = 1;             // lado donde se vio la línea por última vez
uint8_t  ciclosSinLinea = 0;
uint32_t tPerdido = 0;
int8_t   dirBusqueda = 1;

// ensayo de escalón
uint32_t tEscalon = 0;

// calibración
enum FaseCal : uint8_t { CAL_IR, CAL_ESPERA, CAL_A, CAL_B, CAL_C, CAL_FIN };
FaseCal faseCal = CAL_IR;
uint32_t tFase = 0;
float calMinI, calMaxI, calMinD, calMaxD;   // conductancias extremas (pasada A)
struct Pasada { float sPrev, thPrev; bool hayPrev; float cruce, fuerza;
                float n, sx, sy, sxy, sxx; };
Pasada pasB, pasC;

// varios
bool     telemetria = false;
bool     cambiosSinGuardar = false;
uint16_t seq = 0;
uint8_t  divTel = 0;
uint32_t tUltCiclo = 0;
uint16_t atrasos = 0;
uint8_t  causaReset = 0;

// ---------------------------------------------------------------- prototipos
void ciclo();
void medir(bool filtrar);
void escribirServo(float th);
void cambiarEstado(Estado e);
void procesarLinea(char* lin);
void enviarTelemetria();
void pasoCalibracion();
void iniciarCalibracion();
bool cfgValida(const Config& c);
uint16_t crc16(const uint8_t* p, uint16_t n);

// ---------------------------------------------------------------- causa del último reinicio
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

const __FlashStringHelper* nombreEstado(Estado e) {
  switch (e) {
    case REPOSO:  return F("REPOSO");
    case MANUAL:  return F("MANUAL");
    case CALIB:   return F("CALIB");
    case SIGUE:   return F("SIGUE");
    case PERDIDO: return F("PERDIDO");
    case ESCALON: return F("ESCALON");
    default:      return F("FALLA");
  }
}

uint16_t crc16(const uint8_t* p, uint16_t n) {
  uint16_t c = 0xFFFF;
  while (n--) {
    c ^= *p++;
    for (uint8_t i = 0; i < 8; i++) c = (c & 1) ? (c >> 1) ^ 0xA001 : (c >> 1);
  }
  return c;
}

bool cfgValida(const Config& c) {
  if (c.magic != CFG_MAGIC || c.version != CFG_VERSION) return false;
  return crc16((const uint8_t*)&c, offsetof(Config, crc)) == c.crc;
}

bool cargarEEPROM() {
  Config c;
  EEPROM.get(0, c);
  if (!cfgValida(c)) return false;
  cfg = c;
  return true;
}

void guardarEEPROM() {
  cfg.crc = crc16((const uint8_t*)&cfg, offsetof(Config, crc));
  EEPROM.put(0, cfg);
}

void aplicarLeds() {
  analogWrite(PIN_LED_I, (int)cfg.ledi);
  analogWrite(PIN_LED_D, (int)cfg.ledd);
}

// ============================================================================
//  Medición
// ============================================================================
// Promedio de 8 conversiones, descartando la primera tras cambiar de canal
// (el capacitor de muestreo arrastra el valor del canal anterior).
float leerADC(uint8_t pin) {
  analogRead(pin);
  uint16_t suma = 0;
  for (uint8_t i = 0; i < 8; i++) suma += analogRead(pin);
  return suma / 8.0f;
}

// Conductancia relativa del LDR. Divisor 5V–LDR–A–R–GND:
//   raw/1023 = R/(R+Rldr)  ->  1/Rldr = raw / (R·(1023−raw))
// R es constante para cada canal y se cancela al normalizar, así que no hace
// falta conocerla. La conductancia es casi proporcional a la luz.
static inline float conductancia(float raw) {
  raw = limitar(raw, 1.0f, 1022.0f);
  return raw / (1023.5f - raw);
}

// "Negrura" 0..1 de un sensor con su propia calibración (1 = negro).
static inline float negrura(float g, float gBlanco, float gNegro) {
  float d = gBlanco - gNegro;
  if (d < 1e-6f) return 0;
  return 1.0f - limitar((g - gNegro) / d, 0.0f, 1.0f);
}

void medir(bool filtrar) {
  rawI = leerADC(PIN_LDR_I);
  rawD = leerADC(PIN_LDR_D);

  // Sensor desconectado o en corto: el divisor queda pegado a un extremo.
  bool mal = (rawI < 2 || rawI > 1021 || rawD < 2 || rawD > 1021);
  if (mal) { if (ciclosFallaSensor < 255) ciclosFallaSensor++; } else ciclosFallaSensor = 0;
  sensorMal = (ciclosFallaSensor >= 10);

  float nI = negrura(conductancia(rawI), cfg.fBlancoI, cfg.fNegroI);
  float nD = negrura(conductancia(rawD), cfg.fBlancoD, cfg.fNegroD);

  float a = 1.0f;                                  // filtro de 1er orden
  if (filtrar && cfg.tau > 0) a = PERIODO_MS / (cfg.tau + PERIODO_MS);
  bI += a * (nI - bI);
  bD += a * (nD - bD);

  vis = bI + bD;
  sPos = (vis > 0.05f) ? (bD - bI) / vis : 0.0f;
}

// ============================================================================
//  Servo
// ============================================================================
void escribirServo(float th) {
  float fis = th + cfg.trim;
  int us = (int)(US_CENTRO + (fis - 90.0f) * US_POR_GRADO + 0.5f);
  if (us < 544) us = 544;
  if (us > 2400) us = 2400;
  if (us != ultimoUs) { servo.writeMicroseconds(us); ultimoUs = us; }
}

// Lleva thOut hacia thCmd sin superar vmax (°/s) y dentro de [amin, amax].
void moverConLimite() {
  thCmd = limitar(thCmd, cfg.amin, cfg.amax);
  float paso = cfg.vmax * DT;
  thOut += limitar(thCmd - thOut, -paso, paso);
  thOut = limitar(thOut, cfg.amin, cfg.amax);
  escribirServo(thOut);
}

// ============================================================================
//  Estados
// ============================================================================
void cambiarEstado(Estado e) {
  if (e == estado) return;
  estado = e;
  Serial.print(F("S,")); Serial.println(nombreEstado(e));
}

void iniciarSeguimiento() {
  thI = thOut;                 // arranque sin salto: el integrador parte de donde está
  thCmd = thOut;
  ciclosSinLinea = 0;
  if (vis >= cfg.vison) cambiarEstado(SIGUE);
  else {
    // Arranca sin ver la línea: buscar primero hacia donde estaba al calibrar
    tPerdido = millis() - (uint32_t)cfg.thold;
    dirBusqueda = (cfg.thLinea >= thOut) ? 1 : -1;
    cambiarEstado(PERDIDO);
  }
}

void controlPI() {
  // Error en grados: e = θlínea − θbrazo ≈ −s / pendiente
  float e = -sPos / cfg.pendiente;
  if (fabs(sPos) > 0.05f) ladoUltimo = (e > 0) ? 1 : -1;

  // Zona muerta continua (sin salto en el borde)
  if (fabs(e) <= cfg.zm) e = 0;
  else e -= (e > 0 ? cfg.zm : -cfg.zm);
  eGrados = e;

  // Fuera de la zona lineal (s saturado) el error real es mayor que el medido:
  // el integrador avanza ksat veces más rápido para alcanzar la línea.
  float kiEf = cfg.ki * ((fabs(sPos) > 0.9f) ? cfg.ksat : 1.0f);

  float p = cfg.kp * e;
  float objetivo = thI + p;
  // Anti-windup: no integrar si la salida ya está saturada en ese sentido
  bool satAlta = (objetivo >= cfg.amax && e > 0);
  bool satBaja = (objetivo <= cfg.amin && e < 0);
  bool satVel  = (fabs(objetivo - thOut) > cfg.vmax * DT * 1.5f) && ((objetivo - thOut) * e > 0);
  if (!satAlta && !satBaja && !satVel) thI += kiEf * e * DT;
  thI = limitar(thI, cfg.amin, cfg.amax);
  thCmd = limitar(thI + p, cfg.amin, cfg.amax);
}

void ciclo() {
  medir(estado != CALIB);    // en la calibración se usa la lectura sin filtrar

  if (sensorMal && (estado == SIGUE || estado == PERDIDO || estado == CALIB)) {
    Serial.println(F("ERR,sensor,lectura fuera de rango (LDR desconectado o en corto)"));
    thCmd = thOut;
    cambiarEstado(FALLA);
  }

  switch (estado) {
    case REPOSO:
    case FALLA:
      eGrados = 0;
      break;

    case MANUAL:
      if (fabs(thOut - thCmd) < 0.05f) cambiarEstado(REPOSO);
      break;

    case SIGUE:
      if (vis < cfg.visoff) {
        if (++ciclosSinLinea >= 5) {                 // 100 ms sin línea
          tPerdido = millis(); dirBusqueda = ladoUltimo; thCmd = thOut;
          cambiarEstado(PERDIDO);
        }
      } else {
        ciclosSinLinea = 0;
        controlPI();
      }
      break;

    case PERDIDO:
      eGrados = 0;
      if (vis >= cfg.vison) { iniciarSeguimiento(); break; }
      if (millis() - tPerdido >= (uint32_t)cfg.thold) {
        thCmd = thOut + dirBusqueda * cfg.vbusq * DT;  // búsqueda lenta
        if (thCmd >= cfg.amax) { thCmd = cfg.amax; dirBusqueda = -1; }
        if (thCmd <= cfg.amin) { thCmd = cfg.amin; dirBusqueda = 1; }
      } else {
        thCmd = thOut;                                  // espera quieto
      }
      break;

    case ESCALON:
      if (millis() - tEscalon >= 2000) { cambiarEstado(REPOSO); Serial.println(F("OK,esc,fin")); }
      break;

    case CALIB:
      pasoCalibracion();
      break;
  }

  if (estado != ESCALON) moverConLimite();

  if (telemetria) {
    bool forzar = (estado == ESCALON);
    if (forzar || ++divTel >= (uint8_t)cfg.teldiv) { divTel = 0; enviarTelemetria(); }
  }
}

// ============================================================================
//  Calibración por barrido
// ============================================================================
//  A: barrido subiendo -> extremos de conductancia de cada sensor (blanco/negro)
//  B: barrido bajando  -> cruce por cero de s y pendiente
//  C: barrido subiendo -> idem; promediar B y C cancela el retardo de las LDR
void reiniciarPasada(Pasada& p) { memset(&p, 0, sizeof(p)); p.cruce = NAN; }

void iniciarCalibracion() {
  faseCal = CAL_IR;
  thCmd = cfg.amin;
  calMinI = calMinD = 1e9f; calMaxI = calMaxD = -1e9f;
  reiniciarPasada(pasB); reiniciarPasada(pasC);
  cambiarEstado(CALIB);
  Serial.println(F("C,INICIO,la linea debe estar bajo el brazo, con blanco a ambos lados"));
}

void acumularPasada(Pasada& p, float th, float s, float v) {
  // Regresión lineal en la zona útil (s no saturado y línea visible)
  if (v >= cfg.vison && fabs(s) < 0.6f) {
    float x = th - 90.0f;
    p.n += 1; p.sx += x; p.sy += s; p.sxy += x * s; p.sxx += x * x;
  }
  // Cruce por cero: el de mayor salto con la línea bien visible
  if (p.hayPrev && v >= cfg.vison && ((p.sPrev < 0 && s >= 0) || (p.sPrev > 0 && s <= 0))) {
    float f = fabs(s - p.sPrev);
    if (f > p.fuerza) {
      p.fuerza = f;
      p.cruce = p.thPrev + (th - p.thPrev) * (0 - p.sPrev) / (s - p.sPrev);
    }
  }
  p.sPrev = s; p.thPrev = th; p.hayPrev = true;
}

float pendientePasada(const Pasada& p) {
  float den = p.n * p.sxx - p.sx * p.sx;
  if (p.n < 4 || fabs(den) < 1e-6f) return NAN;
  return (p.n * p.sxy - p.sx * p.sy) / den;
}

void falloCalibracion(const __FlashStringHelper* motivo) {
  Serial.print(F("ERR,cal,")); Serial.println(motivo);
  // si había una calibración anterior válida, se conserva
  medir(false);
  thCmd = thOut;
  cambiarEstado(REPOSO);
}

void enviarPuntoCal(uint8_t pasada, float th, float s) {
  Serial.print(F("CS,")); Serial.print(pasada); Serial.print(',');
  Serial.print(th, 2); Serial.print(','); Serial.print(s, 3); Serial.print(',');
  Serial.print(rawI, 0); Serial.print(','); Serial.println(rawD, 0);
}

void pasoCalibracion() {
  float gI = conductancia(rawI), gD = conductancia(rawD);
  switch (faseCal) {
    case CAL_IR:                        // ir al extremo inferior
      thCmd = cfg.amin;
      if (fabs(thOut - cfg.amin) < 0.05f) { faseCal = CAL_ESPERA; tFase = millis(); }
      break;

    case CAL_ESPERA:                    // dejar que las LDR se asienten
      if (millis() - tFase >= 500) { faseCal = CAL_A; Serial.println(F("C,PASADA,A")); }
      break;

    case CAL_A:
      if (gI < calMinI) calMinI = gI;
      if (gI > calMaxI) calMaxI = gI;
      if (gD < calMinD) calMinD = gD;
      if (gD > calMaxD) calMaxD = gD;
      if (seq % 2 == 0) enviarPuntoCal(0, thOut, 0);
      thCmd = thOut + cfg.vcal * DT;
      if (thCmd >= cfg.amax) {
        thCmd = cfg.amax;
        // Contraste mínimo: el blanco debe conducir al menos 1,3 veces más que el negro
        if (calMaxI < 1.3f * calMinI || calMaxD < 1.3f * calMinD) {
          falloCalibracion(F("contraste insuficiente: la linea no paso bajo los dos sensores o falta luz"));
          return;
        }
        // aplicar la normalización nueva para las pasadas B y C
        cfg.fBlancoI = calMaxI; cfg.fNegroI = calMinI;
        cfg.fBlancoD = calMaxD; cfg.fNegroD = calMinD;
        faseCal = CAL_B; Serial.println(F("C,PASADA,B"));
      }
      break;

    case CAL_B:
    case CAL_C: {
      Pasada& p = (faseCal == CAL_B) ? pasB : pasC;
      // lectura sin filtrar ya normalizada con la calibración nueva
      acumularPasada(p, thOut, sPos, vis);
      if (seq % 2 == 0) enviarPuntoCal(faseCal == CAL_B ? 1 : 2, thOut, sPos);
      if (faseCal == CAL_B) {
        thCmd = thOut - cfg.vcal * DT;
        if (thCmd <= cfg.amin) { thCmd = cfg.amin; faseCal = CAL_C; Serial.println(F("C,PASADA,C")); }
      } else {
        thCmd = thOut + cfg.vcal * DT;
        if (thCmd >= cfg.amax) { thCmd = cfg.amax; faseCal = CAL_FIN; }
      }
      break;
    }

    case CAL_FIN: {
      float kB = pendientePasada(pasB), kC = pendientePasada(pasC);
      if (isnan(pasB.cruce) || isnan(pasC.cruce)) { falloCalibracion(F("no se encontro el centro de la linea")); return; }
      if (isnan(kB) || isnan(kC) || kB * kC <= 0) { falloCalibracion(F("pendiente inconsistente entre pasadas")); return; }
      float k = 0.5f * (kB + kC);
      if (fabs(k) < 0.05f || fabs(k) > 5.0f) { falloCalibracion(F("pendiente fuera de rango")); return; }
      if (fabs(pasB.cruce - pasC.cruce) > 8.0f) { falloCalibracion(F("cruces muy distintos (algo se movio?)")); return; }
      cfg.pendiente = k;
      cfg.thLinea = 0.5f * (pasB.cruce + pasC.cruce);
      cfg.calOk = 1;
      cambiosSinGuardar = true;
      Serial.print(F("C,OK,linea=")); Serial.print(cfg.thLinea, 2);
      Serial.print(F(",pendiente=")); Serial.print(k, 4);
      Serial.print(F(",retardo_grados=")); Serial.print(0.5f * fabs(pasB.cruce - pasC.cruce), 2);
      Serial.print(F(",contrasteI=")); Serial.print(calMaxI / calMinI, 2);
      Serial.print(F(",contrasteD=")); Serial.println(calMaxD / calMinD, 2);
      Serial.println(F("OK,cal (usa 'save' para guardarla)"));
      thCmd = cfg.thLinea;
      bI = bD = 0; medir(false);
      cambiarEstado(MANUAL);
      break;
    }
  }
}

// ============================================================================
//  Salida de datos
// ============================================================================
void enviarTelemetria() {
  // T,seq,ms,estado,rawI,rawD,bI,bD,s,e,th,thI
  Serial.print(F("T,"));  Serial.print(seq);         Serial.print(',');
  Serial.print(millis()); Serial.print(',');
  Serial.print(nombreEstado(estado)); Serial.print(',');
  Serial.print(rawI, 0);  Serial.print(',');
  Serial.print(rawD, 0);  Serial.print(',');
  Serial.print(bI, 3);    Serial.print(',');
  Serial.print(bD, 3);    Serial.print(',');
  Serial.print(sPos, 3);  Serial.print(',');
  Serial.print(eGrados, 2); Serial.print(',');
  Serial.print(thOut, 2); Serial.print(',');
  Serial.println(thI, 2);
}

void enviarParametro(uint8_t i) {
  ParamDef d; memcpy_P(&d, &PARAMS[i], sizeof(d));
  float v = *(float*)((uint8_t*)&cfg + d.ofs);
  Serial.print(F("P,")); Serial.print(d.nombre); Serial.print(',');
  Serial.print(v, 4); Serial.print(','); Serial.print(d.minv, 4); Serial.print(',');
  Serial.println(d.maxv, 4);
}

void enviarCalibracion() {
  Serial.print(F("K,")); Serial.print(cfg.calOk); Serial.print(',');
  Serial.print(cfg.fBlancoI, 4); Serial.print(','); Serial.print(cfg.fNegroI, 4); Serial.print(',');
  Serial.print(cfg.fBlancoD, 4); Serial.print(','); Serial.print(cfg.fNegroD, 4); Serial.print(',');
  Serial.print(cfg.pendiente, 4); Serial.print(','); Serial.println(cfg.thLinea, 2);
}

void enviarStatus() {
  Serial.print(F("ST,")); Serial.print(nombreEstado(estado)); Serial.print(',');
  Serial.print(cfg.calOk); Serial.print(','); Serial.print(cambiosSinGuardar ? 1 : 0); Serial.print(',');
  Serial.print(atrasos); Serial.print(','); Serial.print(thOut, 2); Serial.print(',');
  Serial.println(telemetria ? 1 : 0);
}

void enviarAyuda() {
  Serial.println(F("I,ordenes: get | set <nom> <val> | run | stop | cal | goto <g> | esc <g>"));
  Serial.println(F("I,         save | load | defaults | tel 1|0 | status"));
}

// ============================================================================
//  Órdenes por puerto serie (línea completa, sin bloquear)
// ============================================================================
char bufLinea[40];
uint8_t lenLinea = 0;
bool lineaDescartada = false;

void leerSerie() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineaDescartada) { Serial.println(F("ERR,linea demasiado larga")); lineaDescartada = false; }
      else if (lenLinea > 0) { bufLinea[lenLinea] = 0; procesarLinea(bufLinea); }
      lenLinea = 0;
    } else if (!lineaDescartada) {
      if (lenLinea < sizeof(bufLinea) - 1) bufLinea[lenLinea++] = c;
      else { lineaDescartada = true; lenLinea = 0; }
    }
  }
}

bool leerNumero(const char* tok, float& v) {
  if (!tok) return false;
  char* fin;
  v = (float)strtod(tok, &fin);
  return fin != tok && *fin == 0;
}

void procesarLinea(char* lin) {
  for (char* p = lin; *p; p++) if (*p >= 'A' && *p <= 'Z') *p += 32;   // minúsculas
  char* cmd = strtok(lin, " \t");
  if (!cmd) return;
  char* a1 = strtok(NULL, " \t");
  char* a2 = strtok(NULL, " \t");
  float v;

  // Órdenes permitidas en cualquier momento
  if (!strcmp(cmd, "stop")) {
    thCmd = thOut; cambiarEstado(REPOSO); Serial.println(F("OK,stop")); return;
  }
  if (!strcmp(cmd, "get")) {
    for (uint8_t i = 0; i < N_PARAMS; i++) enviarParametro(i);
    enviarCalibracion(); enviarStatus(); Serial.println(F("END")); return;
  }
  if (!strcmp(cmd, "status")) { enviarStatus(); return; }
  if (!strcmp(cmd, "?") || !strcmp(cmd, "ayuda")) { enviarAyuda(); return; }
  if (!strcmp(cmd, "tel")) {
    if (!leerNumero(a1, v)) { Serial.println(F("ERR,tel,usar tel 1 o tel 0")); return; }
    telemetria = (v != 0); divTel = 0;
    Serial.print(F("OK,tel,")); Serial.println(telemetria ? 1 : 0); return;
  }

  // El resto no se acepta durante la calibración o un ensayo
  if (estado == CALIB || estado == ESCALON) { Serial.print(F("ERR,")); Serial.print(cmd); Serial.println(F(",ocupado (usar stop)")); return; }

  if (!strcmp(cmd, "set")) {
    if (!a1 || !leerNumero(a2, v)) { Serial.println(F("ERR,set,usar set <nombre> <valor>")); return; }
    for (uint8_t i = 0; i < N_PARAMS; i++) {
      ParamDef d; memcpy_P(&d, &PARAMS[i], sizeof(d));
      if (strcmp(a1, d.nombre)) continue;
      if (v < d.minv || v > d.maxv) { Serial.print(F("ERR,set,")); Serial.print(d.nombre); Serial.println(F(",fuera de rango")); enviarParametro(i); return; }
      float* dst = (float*)((uint8_t*)&cfg + d.ofs);
      float viejo = *dst; *dst = v;
      if (cfg.visoff >= cfg.vison) { *dst = viejo; Serial.println(F("ERR,set,visoff debe ser menor que vison")); enviarParametro(i); return; }
      if (!strcmp(d.nombre, "ledi") || !strcmp(d.nombre, "ledd")) aplicarLeds();
      if (!strcmp(d.nombre, "trim")) escribirServo(thOut);
      if (!strcmp(d.nombre, "amin") || !strcmp(d.nombre, "amax")) thI = limitar(thI, cfg.amin, cfg.amax);
      cambiosSinGuardar = true;
      enviarParametro(i);
      return;
    }
    Serial.print(F("ERR,set,parametro desconocido: ")); Serial.println(a1); return;
  }
  if (!strcmp(cmd, "run")) {
    if (!cfg.calOk) { Serial.println(F("ERR,run,sin calibrar (usar cal)")); return; }
    if (sensorMal) { Serial.println(F("ERR,run,sensor fuera de rango")); return; }
    iniciarSeguimiento(); Serial.println(F("OK,run")); return;
  }
  if (!strcmp(cmd, "cal")) { iniciarCalibracion(); return; }
  if (!strcmp(cmd, "goto")) {
    if (!leerNumero(a1, v)) { Serial.println(F("ERR,goto,usar goto <grados>")); return; }
    thCmd = limitar(v, cfg.amin, cfg.amax); cambiarEstado(MANUAL);
    Serial.print(F("OK,goto,")); Serial.println(thCmd, 2); return;
  }
  if (!strcmp(cmd, "esc")) {
    if (!leerNumero(a1, v) || fabs(v) < 0.5f || fabs(v) > 20) { Serial.println(F("ERR,esc,usar esc <grados> (0,5 a 20)")); return; }
    float destino = limitar(thOut + v, cfg.amin, cfg.amax);
    Serial.print(F("OK,esc,")); Serial.print(thOut, 2); Serial.print(','); Serial.println(destino, 2);
    thCmd = thOut = destino; escribirServo(thOut);     // salto brusco, sin rampa
    tEscalon = millis(); cambiarEstado(ESCALON); return;
  }
  if (!strcmp(cmd, "save")) { guardarEEPROM(); cambiosSinGuardar = false; Serial.println(F("OK,save")); return; }
  if (!strcmp(cmd, "load")) {
    if (cargarEEPROM()) { aplicarLeds(); cambiosSinGuardar = false; thCmd = thOut; cambiarEstado(REPOSO); Serial.println(F("OK,load")); }
    else Serial.println(F("ERR,load,EEPROM sin datos validos"));
    return;
  }
  if (!strcmp(cmd, "defaults")) {
    cargarDefaults(); aplicarLeds(); cambiosSinGuardar = true; thCmd = thOut; cambiarEstado(REPOSO);
    Serial.println(F("OK,defaults (sin calibrar; 'save' para guardar)")); return;
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

  pinMode(PIN_LED_I, OUTPUT); pinMode(PIN_LED_D, OUTPUT);
  aplicarLeds();

  // Escribir el pulso ANTES de attach: el servo arranca directo en el centro
  thOut = thCmd = thI = 90;
  escribirServo(thOut);
  servo.attach(PIN_SERVO, 544, 2400);

  delay(300);                  // dejar encender los LED y asentarse las LDR
  medir(false);

  Serial.print(F("BOOT,seguidor," FW_VERSION ","));
  if (causaReset & 0x08)      Serial.print(F("watchdog"));
  else if (causaReset & 0x04) Serial.print(F("caida de tension (brown-out)"));
  else if (causaReset & 0x02) Serial.print(F("boton reset o apertura del puerto"));
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
  if (ahora - tUltCiclo >= PERIODO_MS) {
    tUltCiclo += PERIODO_MS;
    if (ahora - tUltCiclo > 5 * PERIODO_MS) { tUltCiclo = ahora; atrasos++; }   // se atrasó: no recuperar a los saltos
    seq++;
    ciclo();
  }
#if USAR_WATCHDOG && defined(__AVR__)
  wdt_reset();
#endif
}
