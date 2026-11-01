// Simulación física para probar el firmware banco_pid_v3 en la PC (no se carga en la placa).
// Modelo:
//   - servo del brazo SG90: latencia de trama, zona muerta, 1er orden, velocidad máxima, juego 0,8°
//   - brazo de 120 mm con el horn montado 2,5° corrido
//   - dos LDR lentos y DISTINTOS (resistencias, divisores y LED desparejos, uno casi saturado)
//   - línea radial de 19 mm sobre el sector; el sector lo gira el servo perturbador por un
//     piñón: relación REAL 7,2 (el firmware arranca suponiendo 7,375), sentido INVERTIDO y
//     el horn del perturbador corrido 1,5° -> la calibración "calp" tiene que descubrirlo
//   - ruido del ADC y puerto serie a 115200 baud con buffer de 64 bytes
#include "mock/Arduino.h"
#include "mock/Servo.h"
#include "mock/EEPROM.h"
#include <vector>
#include <functional>
#include <random>
#include <algorithm>

static uint64_t t_us = 0;
uint32_t millis() { return (uint32_t)(t_us / 1000); }
static void avanzar(uint64_t dus);
void delay(unsigned long ms) { avanzar((uint64_t)ms * 1000); }
void pinMode(uint8_t, uint8_t) {}

// ------------------------------------------------------------ servo genérico
struct ServoSim {
  std::deque<std::pair<double, double>> cola;
  double usObj = 1472, th = 90, salida = 90, juego, tau, vmax;
  bool moviendo = false;
  ServoSim(double j, double t, double v) : juego(j), tau(t), vmax(v) {}
  void paso(double dt, double t, double lag) {
    while (!cola.empty() && cola.front().first <= t) { usObj = cola.front().second; cola.pop_front(); }
    double obj = (usObj - 1472.0) / ((2400.0 - 544.0) / 180.0) + 90.0;
    double err = obj - th;
    if (!moviendo && fabs(err) > 0.5) moviendo = true;            // zona muerta
    if (moviendo && fabs(err) < 0.15) moviendo = false;
    if (moviendo) th += std::max(-vmax, std::min(vmax, err / (tau * lag))) * dt;
    if (th - salida > juego / 2) salida = th - juego / 2;          // juego de engranajes
    if (salida - th > juego / 2) salida = th + juego / 2;
  }
};

// ------------------------------------------------------------ planta
struct Planta {
  double L = 120, medio = 10, r = 4.25, ancho = 19;
  double reflBlanco = 0.85, reflNegro = 0.08;
  int pwmI = 255, pwmD = 255; double efI = 1.6, efD = 0.65;      // LED izquierdo más brillante
  double R10I = 10000, R10D = 20000, gamma = 0.7, RdivI = 4700, RdivD = 4700;
  double gI = 1e-4, gD = 1e-4;
  ServoSim brazo{0.8, 0.025, 400}, pert{1.0, 0.015, 500};
  double offsetMontaje = 2.5;                                    // horn del brazo corrido
  double relReal = 7.2, sigReal = -1, phi0 = 1.5;                // perturbador real
  double radial = 0;                                             // >0: LDR uno detrás del otro (mal armado)
  double lag = 1.0;
  std::function<double(double)> extra = [](double) { return 0.0; };   // movimiento externo de la línea (°)
  bool ldrDesconectadoI = false;
  std::mt19937 rng{1234};
  std::normal_distribution<double> ruido{0.0, 0.8};

  double fraccionNegra(double d) {
    int N = 40; double acc = 0, tot = 0;
    for (int i = 0; i < N; i++) {
      double u = -r + (i + 0.5) * 2 * r / N, w = sqrt(std::max(0.0, r * r - u * u));
      tot += w; if (fabs(d + u) <= ancho / 2) acc += w;
    }
    return acc / tot;
  }
  double lux(double d, int pwm, double ef) {
    double f = fraccionNegra(d);
    return 120.0 * (f * reflNegro + (1 - f) * reflBlanco) * ef * pwm / 255.0 + 1.0;
  }
  double phiLinea(double t) { return phi0 + sigReal * (pert.salida - 90.0) / relReal + extra(t); }
  double thLineaReal(double t) { return 90.0 + phiLinea(t) - offsetMontaje; }   // ángulo de servo con la línea centrada
  void paso(double dt, double t) {
    brazo.paso(dt, t, lag); pert.paso(dt, t, 1.0);
    double a = (brazo.salida + offsetMontaje - 90.0) * M_PI / 180.0, f = phiLinea(t) * M_PI / 180.0;
    double base = L * sin(a - f), c = medio * cos(a - f);
    double dI = base + c * (1 - radial), dD = base - c * (1 - radial);
    double gIo = 1.0 / (R10I * pow(lux(dI, pwmI, efI) / 10.0, -gamma));
    double gDo = 1.0 / (R10D * pow(lux(dD, pwmD, efD) / 10.0, -gamma));
    gI += (gIo - gI) * dt / ((gIo > gI ? 0.020 : 0.035) * lag);
    gD += (gDo - gD) * dt / ((gDo > gD ? 0.020 : 0.035) * lag);
  }
  int adc(bool izq) {
    if (izq && ldrDesconectadoI) return 0;
    double R = 1.0 / (izq ? gI : gD), Rd = izq ? RdivI : RdivD;
    return (int)std::max(0.0, std::min(1023.0, round(1023.0 * Rd / (Rd + R) + ruido(rng))));
  }
} P;

static uint64_t t_planta = 0;
static void actualizarPlanta() { while (t_planta + 250 <= t_us) { t_planta += 250; P.paso(250e-6, t_planta * 1e-6); } }
void sim_servo_us(int pin, int us) {
  ServoSim& s = (pin == 10) ? P.pert : P.brazo;
  s.cola.push_back({t_us * 1e-6 + 0.010 * (pin == 10 ? 1.0 : P.lag), (double)us});
}
int analogRead(uint8_t pin) { avanzar(112); return P.adc(pin == A0); }
void analogWrite(uint8_t pin, int v) { if (pin == 5) P.pwmI = v; if (pin == 6) P.pwmD = v; }

// ------------------------------------------------------------ puerto serie
struct Byte { uint64_t t; char c; };
static std::deque<Byte> enCamino;
static std::deque<char> rx;
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
static uint64_t bytesTx = 0;
void SimSerial::write_str(const std::string& s) {
  bytesTx += s.size();
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
  for (char c : s) { t += 87; enCamino.push_back({t, c}); }
  tLibreTx = t;
}
static void avanzar(uint64_t dus) { t_us += dus; actualizarPlanta(); moverRx(); }

// ------------------------------------------------------------ firmware
#include "../banco_pid_v3/banco_pid_v3.ino"

// ------------------------------------------------------------ registro y utilidades
struct Muestra { double t, thBrazo, thLinea; std::string est; };
static std::vector<Muestra> reg;
static void correr(double hasta) {
  uint64_t fin = (uint64_t)(hasta * 1e6), prox = t_us;
  while (t_us < fin) {
    loop(); avanzar(30);
    if (t_us >= prox) { prox += 5000; reg.push_back({t_us * 1e-6, P.brazo.salida, P.thLineaReal(t_us * 1e-6), (const char*)nombreEstado(estado)}); }
  }
}
static double ahora() { return t_us * 1e-6; }
struct Metrica { double errMedio, errMax, pp, rms; };
static Metrica ventana(double t0, double t1) {
  double s = 0, s2 = 0, mx = 0, lo = 1e9, hi = -1e9; int n = 0;
  for (auto& m : reg) if (m.t >= t0 && m.t <= t1) {
    double e = m.thBrazo - m.thLinea; s += e; s2 += e * e; mx = std::max(mx, fabs(e));
    lo = std::min(lo, m.thBrazo); hi = std::max(hi, m.thBrazo); n++;
  }
  if (!n) return {0, 0, 0, 0};
  return {s / n, mx, hi - lo, sqrt(s2 / n)};
}
static int fallas = 0;
static void chequear(bool ok, const char* que) { printf("  %s %s\n", ok ? "OK  " : "FALLA", que); if (!ok) fallas++; }
static bool contiene(const std::string& pref, double desde) {
  for (auto& l : salida) if (l.t >= desde && l.s.rfind(pref, 0) == 0) return true;
  return false;
}
static std::string ultima(const std::string& pref) {
  for (auto it = salida.rbegin(); it != salida.rend(); ++it) if (it->s.rfind(pref, 0) == 0) return it->s;
  return "";
}
static std::vector<double> campos(const std::string& s) {
  std::vector<double> v; size_t i = 0;
  while (i <= s.size()) { size_t j = s.find(',', i); if (j == std::string::npos) j = s.size(); v.push_back(atof(s.substr(i, j - i).c_str())); i = j + 1; }
  return v;
}
// Métricas de un escalón calculadas desde la telemetría (igual que el panel)
struct MetEsc { double sobre, tEst, eFin; int nT; };
static MetEsc metricasEnsayo(double desde) {
  // T,seq,ms,estado,rawI,rawD,cI,cD,s,p,ref,e,P,I,D,u,th,L
  double t0 = -1, A = 0; int tipo = 0;
  for (auto& l : salida) if (l.t >= desde && l.s.rfind("E,inicio", 0) == 0) { auto c = campos(l.s); tipo = (int)c[2]; A = c[3]; }
  int seqDisp = -1;
  for (auto& l : salida) if (l.t >= desde && l.s.rfind("E,disparo", 0) == 0) seqDisp = atoi(l.s.substr(10).c_str());
  std::vector<std::pair<double, double>> x;   // (t, posición seguida)
  double L0 = 0, p0 = 0; int nT = 0;
  for (auto& l : salida) {
    if (l.t < desde || l.s.rfind("T,", 0) != 0 || l.s.find("ENSAYO") == std::string::npos) continue;
    nT++;
    size_t k = l.s.find(',', 2); int sq = atoi(l.s.substr(2, k - 2).c_str());
    auto c = campos(l.s);   // c[9] = p, c[17] = L, c[2] = ms
    if (sq == seqDisp) { t0 = c[2] / 1000.0; L0 = c[17]; p0 = c[9]; }
    if (seqDisp >= 0 && sq >= seqDisp) x.push_back({c[2] / 1000.0 - t0, tipo == 0 ? -c[9] : c[9] - p0});
  }
  // pesc: el error medido (-p) debe volver a 0;  resc/sesc: p debe llegar a A
  double obj = (tipo == 0) ? 0 : A, sobre = 0, tEst = 0, eFin = 0; int nf = 0;
  double band = std::max(0.05 * fabs(A), 0.25), tEnd = x.empty() ? 0 : x.back().first;
  for (auto& q : x) {
    double dev = (q.second - obj) * (A > 0 ? 1 : -1) * (tipo == 0 ? -1 : 1);
    if (q.first > 0.05) sobre = std::max(sobre, dev);
    if (fabs(q.second - obj) > band) tEst = q.first;
    if (q.first > 0.75 * tEnd) { eFin += q.second - obj; nf++; }
  }
  (void)L0;
  return {sobre / fabs(A) * 100, tEst, nf ? eFin / nf : 0, nT};
}

static void escribirDemo() {
  FILE* f = fopen("demo.txt", "w");
  for (auto& l : salida) fprintf(f, "%.3f\t%s\n", l.t, l.s.c_str());
  fclose(f);
  printf("demo.txt: %zu lineas\n", salida.size());
}

int main() {
  memset(sim_eeprom, 0xFF, sizeof sim_eeprom);
  if (getenv("LAG")) P.lag = atof(getenv("LAG"));
  bool demo = getenv("DEMO") != nullptr;

  printf("== ARRANQUE ==\n");
  eco = !demo; setup(); eco = false;
  chequear(contiene("BOOT,banco_pid,3.0", 0), "mensaje de arranque");
  correr(0.5);
  printf("  crudo sobre blanco: izq=%.0f der=%.0f (izquierdo casi saturado, a proposito)\n", rawI, rawD);
  if (demo) { enviar("tel 1\nget\n"); correr(1.5); }

  printf("== CALIBRACION DEL BRAZO (cal) ==\n");
  double t0 = ahora();
  enviar("cal\n"); correr(t0 + 40);
  if (demo) { enviar("get\n"); correr(ahora() + 0.5); }
  printf("  %s\n  %s\n", ultima("C,DIAG").c_str(), ultima("C,OK").c_str());
  chequear(cfg.calOk == 1, "calibracion exitosa");
  auto dg = campos(ultima("C,DIAG"));
  chequear(dg.size() >= 9 && fabs(dg[8]) > 6 && fabs(dg[8]) < 13, "desfase entre LDR medido (~9,5°)");
  double thL = P.thLineaReal(ahora());
  printf("  linea real en %.2f°, calibracion dice %.2f°\n", thL, cfg.thLinea);
  chequear(fabs(cfg.thLinea - thL) < 1.0, "centro calibrado a menos de 1° del real");

  printf("== CALIBRACION DEL PERTURBADOR (calp) ==\n");
  t0 = ahora(); enviar("calp\n"); correr(t0 + 25);
  if (demo) { enviar("get\n"); correr(ahora() + 0.5); }
  printf("  %s\n", ultima("CP,OK").empty() ? ultima("ERR").c_str() : ultima("CP,OK").c_str());
  chequear(cfg.calpOk == 1, "calp exitosa");
  chequear((int)cfg.psig == -1, "detecto el sentido invertido del engranaje");
  chequear(fabs(cfg.prel - P.relReal) / P.relReal < 0.04, "corrigio la relacion real (7,2) a menos de 4 %");
  correr(ahora() + 1);
  double lineaEn90 = P.thLineaReal(ahora()) + P.offsetMontaje;   // ángulo del brazo físico (sin trim) con línea 0
  printf("  con lin=0 la linea queda en %.2f° del servo del brazo (se pidio 90° + horn %.1f°)\n", P.thLineaReal(ahora()), -P.offsetMontaje);
  chequear(fabs(P.thLineaReal(ahora()) - 90.0) < 0.8, "cero de la linea bajo el brazo en 90° (servo)");
  (void)lineaEn90;

  printf("== PI POR DEFECTO, LINEA QUIETA ==\n");
  enviar("goto 82\n"); correr(ahora() + 1.0);
  t0 = ahora(); enviar("run\n"); correr(t0 + 6);
  Metrica m = ventana(t0 + 3, t0 + 6);
  printf("  error medio %.2f°, pico a pico %.2f°\n", m.errMedio, m.pp);
  chequear(fabs(m.errMedio) < 0.6 && m.pp < 1.2, "centrado y sin bucle");

  printf("== MODO P: ERROR DE REGIMEN (didactico) ==\n");
  enviar("lin 4\n"); correr(ahora() + 3);
  enviar("set modo 1\nset kp 1\n"); correr(ahora() + 4);
  m = ventana(ahora() - 1.5, ahora());
  printf("  P puro kp=1 con la linea a 4° de u0: error de regimen %.2f° (teoria ~ -2°)\n", m.errMedio);
  chequear(fabs(m.errMedio) > 1.0, "P puro deja error de regimen");
  enviar("set modo 2\nset kp 0.4\n"); correr(ahora() + 4);
  m = ventana(ahora() - 1.5, ahora());
  printf("  PI: error %.2f°\n", m.errMedio);
  chequear(fabs(m.errMedio) < 0.6, "PI elimina el error de regimen");

  printf("== PID CON DERIVATIVO ==\n");
  enviar("set modo 4\nset kd 0.02\nset kdf 40\n"); correr(ahora() + 4);
  m = ventana(ahora() - 2, ahora());
  printf("  PID: error %.2f°, pico a pico %.2f°\n", m.errMedio, m.pp);
  chequear(fabs(m.errMedio) < 0.6 && m.pp < 1.5, "PID estable");
  enviar("set modo 2\nset kd 0\nlin 0\n"); correr(ahora() + 3);

  const char* nombres[] = {"pesc", "resc", "sesc", "pram", "psen"};
  struct Caso { const char* orden; double espera; } casos[] = {
    {"ens pesc 3\n", 6}, {"ens pesc -3\n", 6}, {"ens resc 1.5\n", 6}, {"ens pram 4 2\n", 6}, {"ens psen 2 0,5\n", 12}};
  printf("== ENSAYOS CON EL LAZO CERRADO ==\n");
  enviar("tel 1\n"); correr(ahora() + 0.3);
  for (auto& c : casos) {
    t0 = ahora(); enviar(c.orden); correr(t0 + c.espera);
    bool fin = contiene("E,fin", t0);
    MetEsc me = metricasEnsayo(t0);
    auto ini = campos(ultima("E,inicio"));
    int tipo = (int)ini[2];
    printf("  %-16s fin=%s  telemetria=%d lineas", c.orden, fin ? "si" : "NO", me.nT);
    if (tipo <= 2) printf("  sobrepaso %.0f %%  establecimiento %.2f s  error final %.2f°", me.sobre, me.tEst, me.eFin);
    printf("\n");
    chequear(fin && me.nT > 0.9 * 50 * (ini[5] + ini[6]), nombres[tipo]);
    if (tipo <= 1) chequear(fabs(me.eFin) < 0.5, "  ...llega sin error final");
    correr(ahora() + 1);
  }
  enviar("lin 0\n"); correr(ahora() + 2);

  printf("== ESCALON DEL SERVO A LAZO ABIERTO (sesc) ==\n");
  enviar("stop\n"); correr(ahora() + 0.3);
  // llegar desde abajo (el juego queda tomado en el sentido del escalón) y quedar 1° antes de la línea
  for (double d : {-2.5, -1.0}) { char b[32]; snprintf(b, sizeof b, "goto %.1f\n", P.thLineaReal(ahora()) + d); enviar(b); correr(ahora() + 1.2); }
  t0 = ahora(); enviar("ens sesc 2\n"); correr(t0 + 6);
  MetEsc me = metricasEnsayo(t0);
  printf("  servo solo: sobrepaso %.0f %%, establecimiento %.2f s, telemetria %d\n", me.sobre, me.tEst, me.nT);
  chequear(contiene("E,fin", t0) && estado == REPOSO, "sesc completo y vuelve a reposo");

  printf("== INTERRUPTORES Y CORRECCION DE LDR ==\n");
  enviar("goto 90\n"); correr(ahora() + 1); enviar("run\n"); correr(ahora() + 2);
  enviar("set aw 0\nset lvel 0\nset zm 0\nset dmed 0\n"); correr(ahora() + 3);
  m = ventana(ahora() - 1.5, ahora());
  chequear(estado == SIGUE && fabs(m.errMedio) < 0.8, "sigue con anti-windup, limite de velocidad y zona muerta apagados");
  enviar("set aw 1\nset lvel 1\nset zm 0.2\nset dmed 1\nset corr 0\n"); correr(ahora() + 3);
  m = ventana(ahora() - 1.5, ahora());
  printf("  corr 0 (sin igualar): error medio %.2f°\n", m.errMedio);
  enviar("set corr 2\nset gi 0.8\nset oi 50\n"); correr(ahora() + 3);
  chequear(fabs(cfg.gi - 0.8f) < 1e-4 && fabs(cfg.oi - 50) < 1e-4 && estado != FALLA, "modo manual (ganancia/offset) aceptado");
  enviar("set corr 1\nset gi 1\nset oi 0\n"); correr(ahora() + 2);

  printf("== PERTURBACION MANUAL CON lin (lazo siguiendo) ==\n");
  t0 = ahora(); enviar("lin 6\n"); correr(t0 + 4);
  m = ventana(ahora() - 1, ahora());
  chequear(fabs(m.errMedio) < 0.6, "sigue la linea movida por el perturbador");
  enviar("lin 0\n"); correr(ahora() + 3);

  if (demo) { enviar("stop\nsave\nstatus\n"); correr(ahora() + 0.5); escribirDemo(); return 0; }
  enviar("tel 0\n"); correr(ahora() + 0.3);

  printf("== ORDENES EN RAFAGA Y COMA DECIMAL ==\n");
  t0 = ahora();
  enviar("set kp 0,35\nset ki 3.5\nstatus\nset zm 0.25\nxyz\nset kp 99\n"
         "esta linea es demasiado larga para el buffer del nano 1234567890\nset tau 30\n");
  correr(t0 + 1);
  chequear(fabs(cfg.kp - 0.35f) < 1e-4 && fabs(cfg.ki - 3.5f) < 1e-4 && fabs(cfg.zm - 0.25f) < 1e-4 && fabs(cfg.tau - 30) < 1e-4,
           "ordenes validas aplicadas (tambien con coma decimal)");
  chequear(contiene("ERR,orden desconocida: xyz", t0) && contiene("ERR,set,kp,fuera de rango", t0) && contiene("ERR,linea demasiado larga", t0),
           "ordenes invalidas rechazadas con mensaje");
  chequear(estado == SIGUE, "el control siguio durante la rafaga");

  printf("== EEPROM ==\n");
  enviar("stop\nsave\n"); correr(ahora() + 0.3);
  Config copia = cfg;
  enviar("set kp 1.2\nload\n"); correr(ahora() + 0.5);
  chequear(fabs(cfg.kp - copia.kp) < 1e-6 && cfg.calOk == 1 && cfg.calpOk == 1 && fabs(cfg.prel - copia.prel) < 1e-6,
           "save/load restaura parametros y ambas calibraciones");
  sim_eeprom[20] ^= 0x55; enviar("load\n"); correr(ahora() + 0.3);
  chequear(contiene("ERR,load,EEPROM sin datos validos", ahora() - 0.3), "EEPROM corrupta detectada por CRC");
  sim_eeprom[20] ^= 0x55;

  printf("== SENSOR DESCONECTADO ==\n");
  enviar("run\n"); correr(ahora() + 1);
  t0 = ahora(); P.ldrDesconectadoI = true; correr(t0 + 1);
  chequear(estado == FALLA && contiene("ERR,sensor", t0), "falla detectada y control detenido");
  P.ldrDesconectadoI = false; enviar("stop\n"); correr(ahora() + 0.5);

  printf("== CABEZAL MAL ARMADO (LDR uno detras del otro) ==\n");
  P.radial = 0.92; t0 = ahora(); enviar("cal\n"); correr(t0 + 40);
  printf("  %s\n  %s\n", ultima("C,DIAG").c_str(), ultima("ERR,cal").c_str());
  chequear(contiene("ERR,cal,desfase", t0), "la calibracion avisa que la cinta/sensores estan mal orientados");
  P.radial = 0;

  double bps = bytesTx / ahora();
  printf("\nciclos atrasados: %u   bytes/s promedio enviados: %.0f (maximo del puerto ~11500)\n", atrasos, bps);
  printf("RESULTADO: %d chequeos fallidos\n", fallas);
  return fallas ? 1 : 0;
}
