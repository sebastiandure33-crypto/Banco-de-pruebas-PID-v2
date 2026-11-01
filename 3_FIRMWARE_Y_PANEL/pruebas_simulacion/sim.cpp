// Simulación física para probar el firmware seguidor_v2 en la PC.
// Modelo: servo SG90 (latencia, zona muerta, velocidad limitada, juego de engranajes),
// brazo de 120 mm, dos LDR lentos y DISTINTOS (R10 y divisor diferentes, LED desparejos),
// línea negra de 19 mm que se mueve, ruido del ADC y puerto serie a 115200 baud con
// buffer de 64 bytes.
#include "mock/Arduino.h"
#include "mock/Servo.h"
#include "mock/EEPROM.h"
#include <vector>
#include <functional>
#include <random>
#include <algorithm>

// ------------------------------------------------------------ reloj
static uint64_t t_us = 0;
uint32_t millis() { return (uint32_t)(t_us / 1000); }
static void avanzar(uint64_t dus);
void delay(unsigned long ms) { avanzar((uint64_t)ms * 1000); }
void pinMode(uint8_t, uint8_t) {}

// ------------------------------------------------------------ planta
struct Planta {
  // geometría
  double L = 120, medio = 10, r = 4.25, ancho = 19;
  double reflBlanco = 0.85, reflNegro = 0.08;
  // LED (PWM y eficiencia distinta por lado)
  int pwmI = 255, pwmD = 255; double efI = 1.0, efD = 0.65;
  // LDR distintos
  double R10I = 10000, R10D = 20000, gamma = 0.7, RdivI = 4700, RdivD = 10000;
  double gI = 1e-4, gD = 1e-4;                 // conductancia actual
  // servo
  std::deque<std::pair<double,double>> colaUs;   // (instante de llegada, pulso)
  double usObj = 1472;
  double thServo = 90, thBrazo = 90, juego = 0.8;
  bool moviendo = false;
  double offsetMontaje = 0;                    // error de montaje del horn
  double lag = 1.0;                            // multiplicador de lentitud (LDR y servo)
  // línea
  std::function<double(double)> yLinea = [](double) { return 0.0; };
  bool ldrDesconectadoI = false;
  std::mt19937 rng{1234};
  std::normal_distribution<double> ruido{0.0, 0.8};

  double fraccionNegra(double ys, double yl) {
    // fracción del círculo visto (radio r) que cae sobre la franja negra
    int N = 40; double acc = 0, tot = 0;
    for (int i = 0; i < N; i++) {
      double u = -r + (i + 0.5) * 2 * r / N;
      double w = sqrt(std::max(0.0, r * r - u * u));
      tot += w;
      if (fabs(ys + u - yl) <= ancho / 2) acc += w;
    }
    return acc / tot;
  }
  double luxSensor(double ys, double yl, int pwm, double ef) {
    double f = fraccionNegra(ys, yl);
    double refl = f * reflNegro + (1 - f) * reflBlanco;
    return 120.0 * refl * ef * pwm / 255.0 + 1.0;
  }
  void paso(double dt, double t) {
    // servo: el pulso llega con ~10 ms de latencia (trama de 20 ms)
    while (!colaUs.empty() && colaUs.front().first <= t) { usObj = colaUs.front().second; colaUs.pop_front(); }
    double obj = (usObj - 1472.0) / ((2400.0 - 544.0) / 180.0) + 90.0;
    double err = obj - thServo;
    if (!moviendo && fabs(err) > 0.5) moviendo = true;      // zona muerta del SG90
    if (moviendo && fabs(err) < 0.15) moviendo = false;
    if (moviendo) {
      double v = err / (0.025 * lag);                        // 1er orden, tau 25 ms
      v = std::max(-400.0, std::min(400.0, v));              // velocidad máx con brazo
      thServo += v * dt;
    }
    // juego entre el eje y el brazo
    if (thServo - thBrazo > juego / 2) thBrazo = thServo - juego / 2;
    if (thBrazo - thServo > juego / 2) thBrazo = thServo + juego / 2;
    // sensores
    double phi = (thBrazo + offsetMontaje - 90.0) * M_PI / 180.0;
    double yh = L * sin(phi), c = medio * cos(phi);
    double yl = yLinea(t);
    double luxI = luxSensor(yh + c, yl, pwmI, efI);
    double luxD = luxSensor(yh - c, yl, pwmD, efD);
    double gIobj = 1.0 / (R10I * pow(luxI / 10.0, -gamma));
    double gDobj = 1.0 / (R10D * pow(luxD / 10.0, -gamma));
    gI += (gIobj - gI) * dt / ((gIobj > gI ? 0.020 : 0.035) * lag);  // LDR: sube rápido, baja lento
    gD += (gDobj - gD) * dt / ((gDobj > gD ? 0.020 : 0.035) * lag);
  }
  int adc(bool izq) {
    if (izq && ldrDesconectadoI) return 0;
    double R = 1.0 / (izq ? gI : gD), Rd = izq ? RdivI : RdivD;
    double v = 1023.0 * Rd / (Rd + R) + ruido(rng);
    return (int)std::max(0.0, std::min(1023.0, round(v)));
  }
  double thLineaReal(double t) { return 90.0 + asin(yLinea(t) / L) * 180.0 / M_PI - offsetMontaje; }
} P;

static uint64_t t_planta = 0;
static void actualizarPlanta() {
  while (t_planta + 250 <= t_us) { t_planta += 250; P.paso(250e-6, t_planta * 1e-6); }
}

void sim_servo_us(int us) { P.colaUs.push_back({t_us * 1e-6 + 0.010 * P.lag, (double)us}); }
int analogRead(uint8_t pin) { avanzar(112); return P.adc(pin == A0); }
void analogWrite(uint8_t pin, int v) { if (pin == 5) P.pwmI = v; if (pin == 6) P.pwmD = v; }

// ------------------------------------------------------------ puerto serie
struct Byte { uint64_t t; char c; };
static std::deque<Byte> enCamino;      // bytes viajando por el cable
static std::deque<char> rx;            // buffer de recepción (64 bytes, como el Nano)
static int desbordes = 0;
static uint64_t tLibreTx = 0;
static void moverRx() {
  while (!enCamino.empty() && enCamino.front().t <= t_us) {
    if (rx.size() < 63) rx.push_back(enCamino.front().c); else desbordes++;
    enCamino.pop_front();
  }
}
int SimSerial::available() { moverRx(); return (int)rx.size(); }
int SimSerial::read() { moverRx(); if (rx.empty()) return -1; char c = rx.front(); rx.pop_front(); return c; }

struct Linea { double t; std::string s; };
static std::vector<Linea> salida;
static std::string acum;
static bool eco = false;
void SimSerial::write_str(const std::string& s) {
  for (char c : s) {
    if (c == '\n') { salida.push_back({t_us * 1e-6, acum}); if (eco) printf("  [%7.3f] %s\n", t_us * 1e-6, acum.c_str()); acum.clear(); }
    else if (c != '\r') acum += c;
  }
}
SimSerial Serial;
uint8_t sim_eeprom[1024];
SimEEPROM EEPROM;

static void enviar(const std::string& s) {
  uint64_t t = std::max(t_us, tLibreTx);
  for (char c : s) { t += 87; enCamino.push_back({t, c}); }   // 115200 baud
  tLibreTx = t;
}

static void avanzar(uint64_t dus) { t_us += dus; actualizarPlanta(); moverRx(); }

// ------------------------------------------------------------ firmware
#include "../seguidor_v2/seguidor_v2.ino"

// ------------------------------------------------------------ registro
struct Muestra { double t, thBrazo, thLinea, thOut, s; std::string est; };
static std::vector<Muestra> reg;
static void correr(double hasta) {
  uint64_t fin = (uint64_t)(hasta * 1e6);
  uint64_t proxReg = t_us;
  while (t_us < fin) {
    loop();
    avanzar(30);
    if (t_us >= proxReg) {
      proxReg += 5000;
      reg.push_back({t_us * 1e-6, P.thBrazo, P.thLineaReal(t_us * 1e-6), thOut, sPos, ""});
      reg.back().est = (const char*)nombreEstado(estado);
    }
  }
}
static double ahora() { return t_us * 1e-6; }

struct Metrica { double errMedio, errMaxAbs, pp, rms, ppCmd; };
static Metrica medirVentana(double t0, double t1) {
  double s = 0, s2 = 0, mx = 0, lo = 1e9, hi = -1e9, lc = 1e9, hc = -1e9; int n = 0;
  for (auto& m : reg) if (m.t >= t0 && m.t <= t1) {
    double e = m.thBrazo - m.thLinea; s += e; s2 += e * e; mx = std::max(mx, fabs(e));
    lo = std::min(lo, m.thBrazo); hi = std::max(hi, m.thBrazo);
    lc = std::min(lc, m.thOut); hc = std::max(hc, m.thOut); n++;
  }
  if (!n) return {0, 0, 0, 0, 0};
  return {s / n, mx, hi - lo, sqrt(s2 / n), hc - lc};
}
static int fallas = 0;
static void chequear(bool ok, const char* que) { printf("  %s %s\n", ok ? "OK  " : "FALLA", que); if (!ok) fallas++; }
static bool salidaContiene(const std::string& pref, double desde) {
  for (auto& l : salida) if (l.t >= desde && l.s.rfind(pref, 0) == 0) return true;
  return false;
}
static std::string ultimaQueEmpieza(const std::string& pref) {
  for (auto it = salida.rbegin(); it != salida.rend(); ++it) if (it->s.rfind(pref, 0) == 0) return it->s;
  return "";
}

static int demo() {
  memset(sim_eeprom, 0xFF, sizeof sim_eeprom);
  P.offsetMontaje = 2.5;
  double yl0 = 120 * sin(3.0 * M_PI / 180);
  P.yLinea = [=](double) { return yl0; };
  setup();
  correr(1.0);
  enviar("tel 1\nget\n"); correr(2.0);
  enviar("cal\n"); correr(ahora() + 26);
  enviar("goto 82\n"); correr(ahora() + 1.5);
  enviar("run\n"); correr(ahora() + 4);
  double t0 = ahora();
  P.yLinea = [=](double t) { return t < t0 + 0.3 ? yl0 : yl0 + 8; };
  correr(t0 + 3);
  t0 = ahora(); double yb = yl0 + 8;
  P.yLinea = [=](double t) { return yb + 25 * sin(25.0 / 25.0 * (t - t0)); };
  correr(t0 + 9);
  t0 = ahora(); double yc = P.yLinea(t0);
  P.yLinea = [=](double t) { return t < t0 + 0.3 ? yc : yl0 - 40; };
  correr(t0 + 5);
  enviar("stop\n"); correr(ahora() + 0.5);
  enviar("esc 6\n"); correr(ahora() + 2.5);
  enviar("save\nstatus\n"); correr(ahora() + 0.5);
  FILE* f = fopen("demo.txt", "w");
  for (auto& l : salida) fprintf(f, "%.3f\t%s\n", l.t, l.s.c_str());
  fclose(f);
  printf("demo: %zu lineas\n", salida.size());
  return 0;
}

int main() {
  if (getenv("DEMO")) return demo();
  memset(sim_eeprom, 0xFF, sizeof sim_eeprom);       // EEPROM virgen
  if (getenv("LAG")) P.lag = atof(getenv("LAG"));
  P.offsetMontaje = 2.5;                             // horn montado 2,5° corrido
  double yl0 = 120 * sin(3.0 * M_PI / 180);          // línea en ~92,5° del servo
  P.yLinea = [=](double) { return yl0; };

  printf("== ARRANQUE ==\n");
  eco = true; setup(); eco = false;
  chequear(salidaContiene("BOOT,seguidor", 0), "mensaje de arranque");
  correr(0.5);
  printf("  RAW sobre blanco/negro (muy distintos a proposito): rawI=%.0f rawD=%.0f\n", rawI, rawD);

  printf("== CALIBRACION ==\n");
  double t0 = ahora();
  enviar("cal\n");
  correr(t0 + 40);
  std::string r = ultimaQueEmpieza("C,OK");
  printf("  %s\n", r.empty() ? ultimaQueEmpieza("ERR").c_str() : r.c_str());
  printf("  [debug] gI %.4f..%.4f  gD %.4f..%.4f\n", calMinI, calMaxI, calMinD, calMaxD);
  chequear(cfg.calOk == 1, "calibracion exitosa");
  double thL = P.thLineaReal(ahora());
  printf("  linea real en %.2f°, calibracion dice %.2f°\n", thL, cfg.thLinea);
  chequear(fabs(cfg.thLinea - thL) < 1.0, "centro calibrado a menos de 1° del real");

  if (getenv("SETS")) { std::string s = getenv("SETS"); for (auto& ch : s) if (ch == ';') ch = '\n'; enviar(s + "\n"); correr(ahora() + 0.5); }
  printf("== SEGUIMIENTO, LINEA QUIETA ==\n");
  enviar("goto 80\n"); correr(ahora() + 1.0);
  t0 = ahora(); enviar("run\n"); correr(t0 + 6);
  Metrica m = medirVentana(t0 + 3, t0 + 6);
  printf("  error medio %.2f°, max %.2f°, oscilacion pico a pico %.2f° (comando %.2f°)\n", m.errMedio, m.errMaxAbs, m.pp, m.ppCmd);
  chequear(fabs(m.errMedio) < 0.6, "centrado (error medio < 0,6°)");
  chequear(m.pp < 1.2, "sin bucle (oscilacion < 1,2° pico a pico, el juego mecanico es 0,8°)");
  double tAsent = -1;
  for (auto& s : reg) if (s.t >= t0 && fabs(s.thBrazo - s.thLinea) < 1.0) { tAsent = s.t - t0; break; }
  printf("  desde 80° llega a <1° de la linea en %.2f s\n", tAsent);

  printf("== ESCALON DE LA LINEA (8 mm) ==\n");
  t0 = ahora();
  P.yLinea = [=](double t) { return t < t0 + 0.2 ? yl0 : yl0 + 8; };
  correr(t0 + 4);
  m = medirVentana(t0 + 2, t0 + 4);
  double sobre = 0, objetivo = P.thLineaReal(ahora()), inicio = P.thLineaReal(t0);
  for (auto& s : reg) if (s.t >= t0) sobre = std::max(sobre, (s.thBrazo - objetivo) * (objetivo > inicio ? 1 : -1));
  printf("  error final %.2f°, sobrepaso %.2f°\n", m.errMedio, sobre);
  chequear(fabs(m.errMedio) < 0.6 && sobre < 2.0, "escalon absorbido sin oscilar");

  printf("== LINEA EN MOVIMIENTO (senoidal) ==\n");
  double vels[3] = {10, 25, 45};
  for (double vmm : vels) {
    t0 = ahora(); double A = 25, w = vmm / A;           // velocidad pico = vmm (mm/s)
    double yb = yl0 + 8;
    P.yLinea = [=](double t) { return yb + A * sin(w * (t - t0)); };
    correr(t0 + 2 * M_PI / w * 2);
    m = medirVentana(t0 + 2 * M_PI / w * 0.5, ahora());
    int perd = 0; for (auto& s : reg) if (s.t >= t0 && s.est == "PERDIDO") perd++;
    printf("  linea a %4.0f mm/s pico: error RMS %.2f°, max %.2f°, tiempo perdido %.2f s\n", vmm, m.rms, m.errMaxAbs, perd * 0.005);
    if (vmm <= 25) chequear(perd == 0 && m.errMaxAbs < 3.5, "sigue la linea sin perderla");
  }

  printf("== LINEA PERDIDA (salto de 45 mm) ==\n");
  t0 = ahora();
  P.yLinea = [=](double t) { return t < t0 + 0.2 ? yl0 : yl0 - 45; };
  correr(t0 + 8);
  bool paso = salidaContiene("S,PERDIDO", t0);
  double tRec = -1;
  for (auto& s : reg) if (s.t > t0 + 0.2 && fabs(s.thBrazo - s.thLinea) < 1.0) { tRec = s.t - t0 - 0.2; break; }
  printf("  paso a PERDIDO: %s, recuperada a <1° en %.2f s\n", paso ? "si" : "no", tRec);
  chequear(paso && tRec > 0 && tRec < 5, "busca y recupera la linea");

  printf("== ORDENES EN RAFAGA (robustez del protocolo) ==\n");
  t0 = ahora();
  enviar("set kp 0.35\nset ki 3.5\nstatus\nset zm 0.25\nxyz\nset kp 99\n"
         "esta linea es demasiado larga para el buffer del nano 1234567890\nset tau 30\n");
  correr(t0 + 1);
  chequear(fabs(cfg.kp - 0.35f) < 1e-4 && fabs(cfg.ki - 3.5f) < 1e-4 && fabs(cfg.zm - 0.25f) < 1e-4 && fabs(cfg.tau - 30) < 1e-4,
           "todas las ordenes validas aplicadas, sin perder letras");
  chequear(salidaContiene("ERR,orden desconocida: xyz", t0), "orden invalida rechazada");
  chequear(salidaContiene("ERR,set,kp,fuera de rango", t0), "valor fuera de rango rechazado");
  chequear(salidaContiene("ERR,linea demasiado larga", t0), "linea larga descartada");
  printf("  bytes perdidos por desborde del buffer: %d\n", desbordes);
  m = medirVentana(t0 + 0.5, t0 + 1);
  chequear(estado == SIGUE && fabs(m.errMedio) < 0.8, "el control siguio funcionando durante la rafaga");

  printf("== AJUSTES EN VIVO Y PARADA/ARRANQUE (el viejo bug del bucle) ==\n");
  t0 = ahora();
  const char* sets[] = {"set kp 0.8\n", "set ki 10\n", "set kp 0.1\n", "set ki 1\n", "set zm 1.5\n",
                        "set zm 0\n", "set vmax 400\n", "set zm 0.2\n", "set vmax 150\n"};
  for (auto s : sets) { enviar(s); correr(ahora() + 0.4); }
  { std::string s = getenv("SETS") ? getenv("SETS") : "set kp 0.3;set ki 4"; for (auto& ch : s) if (ch == ';') ch = '\n'; enviar(s + "\n"); correr(ahora() + 0.4); }
  enviar("stop\n"); correr(ahora() + 0.5);
  enviar("goto 105\n"); correr(ahora() + 1.0);
  double t1 = ahora(); enviar("run\n"); correr(t1 + 5);
  m = medirVentana(t1 + 3, t1 + 5);
  printf("  tras 11 cambios en vivo + stop/goto/run: error medio %.2f°, pico a pico %.2f°\n", m.errMedio, m.pp);
  chequear(fabs(m.errMedio) < 0.6 && m.pp < 1.2, "vuelve a centrar sin reiniciar la placa");

  printf("== GANANCIAS EXAGERADAS (margen de estabilidad) ==\n");
  enviar("set ki 8\nset kp 0.6\n"); correr(ahora() + 3);
  m = medirVentana(ahora() - 2, ahora());
  printf("  ki=8 (el doble): pico a pico %.2f° (comando %.2f°)\n", m.pp, m.ppCmd);
  enviar(getenv("SETS") ? (std::string(getenv("SETS")).replace(std::string(getenv("SETS")).find(';'),1,"\n") + "\n") : std::string("set ki 4\nset kp 0.3\n")); correr(ahora() + 2);

  printf("== EEPROM ==\n");
  enviar("save\n"); correr(ahora() + 0.3);
  Config copia = cfg;
  enviar("set kp 1.2\n"); correr(ahora() + 0.2);
  enviar("load\n"); correr(ahora() + 0.3);
  chequear(fabs(cfg.kp - copia.kp) < 1e-6 && cfg.calOk == 1, "save/load restaura la configuracion y la calibracion");
  sim_eeprom[20] ^= 0x55;                            // corromper un byte
  enviar("load\n"); correr(ahora() + 0.3);
  chequear(salidaContiene("ERR,load,EEPROM sin datos validos", ahora() - 0.3), "EEPROM corrupta detectada por CRC");
  sim_eeprom[20] ^= 0x55;

  printf("== SENSOR DESCONECTADO ==\n");
  enviar("run\n"); correr(ahora() + 1);
  t0 = ahora(); P.ldrDesconectadoI = true; correr(t0 + 1);
  chequear(estado == FALLA && salidaContiene("ERR,sensor", t0), "falla detectada y control detenido");
  P.ldrDesconectadoI = false;
  enviar("stop\n"); correr(ahora() + 0.5);

  printf("== ENSAYO DE ESCALON (identificacion) ==\n");
  enviar("tel 1\n"); correr(ahora() + 0.2);
  t0 = ahora(); enviar("esc 5\n"); correr(t0 + 2.5);
  int nT = 0; for (auto& l : salida) if (l.t >= t0 && l.s.rfind("T,", 0) == 0 && l.s.find("ESCALON") != std::string::npos) nT++;
  printf("  lineas de telemetria durante el ensayo: %d (esperado ~100 a 50 Hz)\n", nT);
  chequear(nT > 90, "registro completo del escalon");
  enviar("tel 0\n"); correr(ahora() + 0.2);

  printf("\nciclos atrasados (lazo de 20 ms): %u\n", atrasos);
  FILE* f = fopen("registro.csv", "w");
  fprintf(f, "t,thBrazo,thLinea,thOut,s,estado\n");
  for (auto& s : reg) fprintf(f, "%.3f,%.3f,%.3f,%.3f,%.3f,%s\n", s.t, s.thBrazo, s.thLinea, s.thOut, s.s, s.est.c_str());
  fclose(f);
  printf("%s: %d chequeos fallidos\n", fallas ? "RESULTADO" : "RESULTADO", fallas);
  return fallas ? 1 : 0;
}
