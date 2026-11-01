"""
Plataforma de la línea para el banco de pruebas PID (se usa junto con seguidor.py).

Piezas:
  P1 pedestal   : se apoya en la mesa; el soporte actual (base con pie) encastra arriba y
                  queda a la altura de la superficie de la línea. Lleva el puente hacia la pista.
  P2 pista      : arco fijo con canal para el sector, escala en grados, alojamiento del piñón
                  y apoyo del soporte del servo perturbador.
  P3 sector     : arco blanco que gira alrededor del eje del servo principal; lleva la cinta
                  negra en un canal radial y una cremallera curva en el borde exterior.
  P4 piñón      : engrana con la cremallera; se monta en el horn del servo perturbador.
  P5 soporte    : puente que sostiene el servo perturbador boca abajo (eje hacia la pista).
                  Se atornilla con ranuras: corriéndolo 3 mm se desengrana el piñón.

Ejecutar:  python3 plataforma.py      -> STL en 1_DISENO_3D/variantes/plataforma/
"""
import math
import os
import sys

import numpy as np
from manifold3d import CrossSection, Manifold

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import seguidor as s  # noqa: E402
from seguidor import cube, cyl, vol  # noqa: E402

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "variantes", "plataforma")

# =====================================================================
#  PARÁMETROS
# =====================================================================
L = 120.0                    # brazo usado
R_FOOT = 34.0                # pie de la base para L = 120

# --- alturas (z = 0 es la mesa)
Z_FLOOR = 3.0                # piso del canal del sector
SECTOR_T = 2.9               # espesor del sector
Z_SURF = 6.0                 # superficie de la línea = cara de abajo del soporte del servo
POCKET_BASE = 2.0            # profundidad del encastre del soporte en el pedestal
CLR = 0.30                   # holgura general de encastres

# --- engranaje (módulo 1,25; ángulo de presión 20°)
MOD = 1.25
Z_RACK = 236                 # dientes de la "rueda" virtual de la cremallera del sector
Z_PIN = 32                   # dientes del piñón
BACKLASH = 0.15              # juego entre dientes (mm, en la circunferencia primitiva)
R_RACK = MOD * Z_RACK / 2    # 147,5
R_PIN = MOD * Z_PIN / 2      # 20,0
X_PIN = R_RACK + R_PIN       # centro del piñón (sobre el eje X, delante del brazo)
RATIO = Z_RACK / Z_PIN       # 7,375 : 1  (180° de servo = 24,4° de línea)
PIN_T = s.ARM_T              # espesor del piñón (mismo hueco de horn que el brazo)

# --- sector
SEC_R0 = 95.0                # radio interior del sector
SEC_HALF = 20.0              # semiángulo del sector
SEC_TRAVEL = 12.5            # giro máximo del sector a cada lado (servo 0..180°)
TAPE_W = 19.6                # canal para cinta de 19 mm
TAPE_D = 0.4
TAPE_R = (98.0, 143.5)

# --- pista
TRACK_R0, TRACK_R1 = 80.0, 162.0
TRACK_HALF = 36.0            # cubre el barrido del LDR exterior a ±30° (34,8°)
CH_HALF = SEC_HALF + SEC_TRAVEL + 1.0
SCALE_R = (86.0, 93.5)
SCALE_HALF = 15

# --- soporte del servo perturbador (servo boca abajo, cuerpo hacia afuera en +X)
PIN_TOP = 6.95                                  # cara superior del piñón
HORN_SEAT2 = PIN_TOP + (s.HORN_HUB_H - s.HORN_PAD_T) + s.HORN_GAP   # extremo del cubo del horn
Z_BOSS2 = HORN_SEAT2 + (s.SV_HORN_SEAT - s.SV_TOP)                  # tapa del servo (mira abajo)
Z_TAB2 = Z_BOSS2 + (s.SV_TOP - s.SV_TAB_Z - s.SV_TAB_T)             # cara de las orejas que apoya
PIN_Z0 = PIN_TOP - PIN_T
BR_PLATE_T = 4.0
LEG_Y0, LEG_Y1 = 23.5, 31.5
BR_X0, BR_X1 = 152.0, 192.0

SEG = 96


# =====================================================================
#  Utilidades
# =====================================================================
def sector_xs(r0, r1, a0, a1, n=240):
    pts = [(r1 * math.cos(math.radians(a)), r1 * math.sin(math.radians(a))) for a in np.linspace(a0, a1, n)]
    pts += [(r0 * math.cos(math.radians(a)), r0 * math.sin(math.radians(a))) for a in np.linspace(a1, a0, n)]
    return CrossSection([pts])


def ext(xs, z0, z1):
    return Manifold.extrude(xs, z1 - z0).translate((0, 0, z0))


def rot(m, a):
    return m.rotate((0, 0, a))


def gear_xs(z, m=MOD, backlash=BACKLASH, phase=0.0, pts_flank=14, teeth=None):
    """Perfil de engranaje recto con evolvente (exterior). teeth = lista de índices a dibujar
    (None = todos). Devuelve el contorno como CrossSection."""
    a = math.radians(20.0)
    rp = m * z / 2
    rb = rp * math.cos(a)
    ra = rp + m
    rf = rp - 1.25 * m
    inv = lambda x: math.tan(x) - x  # noqa: E731
    psi_p = math.pi / (2 * z) - backlash / (2 * rp)
    r_start = max(rb, rf)

    def psi(r):
        al = math.acos(min(1.0, rb / r))
        return psi_p + inv(a) - inv(al)

    rs = np.linspace(r_start, ra, pts_flank)
    pitch = 2 * math.pi / z
    idx = range(z) if teeth is None else teeth
    pts = []
    for k in idx:
        c = k * pitch + math.radians(phase)
        flank_l = [(r, c - psi(r)) for r in rs]
        flank_r = [(r, c + psi(r)) for r in rs[::-1]]
        if rf < rb:
            flank_l = [(rf, c - psi(rb))] + flank_l
            flank_r = flank_r + [(rf, c + psi(rb))]
        tip = [(ra, c - psi(ra) + t * 2 * psi(ra)) for t in np.linspace(0, 1, 4)[1:-1]]
        seq = flank_l + tip + flank_r
        # fondo entre este diente y el siguiente
        if teeth is None:
            g0 = c + psi(r_start) if rf >= rb else c + psi(rb)
            g1 = c + pitch - (psi(r_start) if rf >= rb else psi(rb))
            seq += [(rf, g0 + t * (g1 - g0)) for t in np.linspace(0, 1, 5)[1:-1]]
        pts += seq
    return [(r * math.cos(t), r * math.sin(t)) for r, t in pts], rf


def gear_full(z, z0, z1, phase=0.0):
    pts, _ = gear_xs(z, phase=phase)
    return ext(CrossSection([pts]), z0, z1)


# =====================================================================
#  Piezas
# =====================================================================
def base_huella():
    """Huella de la placa de la base con pie (z < 1), con holgura, como CrossSection."""
    b = s.make_base(R_FOOT) ^ cube(-80, 80, -80, 80, 0, 0.6)
    return b.project().offset(CLR)


def agujeros_base():
    """Centros de los agujeros M3 de la base (para fijarla al pedestal)."""
    _, _, xc, _, _, wy = s.base_dims()
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((xc + sx * s.SV_HOLE_SP / 2, sy * (wy + 2.75)))
    for sa in (-1, 0, 1):
        a = math.radians(sa * (s.FOOT_ANG - 9))
        rr = R_FOOT - 5.5
        pts.append((rr * math.cos(a), rr * math.sin(a)))
    return pts


DOVE = dict(x0=TRACK_R0 - 1.0, x1=TRACK_R0 + 9.0, w0=14.0, w1=20.0)   # cola de milano (vertical)


def dovetail(clr=0.0):
    d = DOVE
    pts = [(d["x0"] - clr, -d["w0"] / 2 - clr), (d["x1"] + clr, -d["w1"] / 2 - clr),
           (d["x1"] + clr, d["w1"] / 2 + clr), (d["x0"] - clr, d["w0"] / 2 + clr)]
    return CrossSection([pts])


def make_pedestal():
    huella = base_huella()
    top = Z_SURF + POCKET_BASE
    body = ext(huella.offset(5.0), 0, top)
    bridge = cube(30.0, math.sqrt(TRACK_R0 ** 2 - 12.0 ** 2) - CLR, -12.0, 12.0, 0, Z_SURF - 0.5)
    tongue = ext(dovetail(), 0, Z_SURF - 0.5)
    solid = body + bridge + tongue
    cuts = ext(huella, Z_SURF, top + 1)
    for (x, y) in agujeros_base():
        cuts += cyl(1.4, Z_SURF - 6.0, top + 1, x=x, y=y, seg=24)     # guía para M3 autorroscante
    cuts += cube(-60, -8, -4.0, 4.0, Z_SURF - 1.2, top + 1)            # salida del cable del servo
    return solid - cuts


def make_track():
    solid = ext(sector_xs(TRACK_R0, TRACK_R1, -TRACK_HALF, TRACK_HALF), 0, Z_SURF)
    solid += cube(X_PIN - 26, BR_X1 + 3, -LEG_Y1 - 2, LEG_Y1 + 2, 0, Z_SURF)    # zona del perturbador
    cuts = ext(sector_xs(SEC_R0 - CLR, R_RACK + MOD + 0.8, -CH_HALF, CH_HALF), Z_FLOOR, Z_SURF + 1)
    cuts += cyl(R_PIN + MOD + 0.9, PIN_Z0 - 0.45, Z_SURF + 1, x=X_PIN, seg=SEG)   # piñón
    cuts += ext(dovetail(CLR / 2), -1, Z_SURF + 1)
    # escala grabada (0,6 mm): cada 1°, larga cada 5°, más larga en 0
    for a in range(-SCALE_HALF, SCALE_HALF + 1):
        ln = 7.5 if a == 0 else (5.5 if a % 5 == 0 else 3.0)
        w = 0.9 if a % 5 == 0 else 0.7
        t = cube(SCALE_R[1] - ln, SCALE_R[1], -w / 2, w / 2, Z_SURF - 0.6, Z_SURF + 1)
        cuts += rot(t, a)
    cuts += cyl(1.0, Z_SURF - 0.6, Z_SURF + 1, x=SCALE_R[0] - 2.5, seg=24)
    # guías M3 para el soporte del perturbador (ranuras en el soporte)
    for x in BR_HOLES_X:
        for sy in (-1, 1):
            cuts += cyl(1.4, Z_SURF - 5.0, Z_SURF + 1, x=x, y=sy * (LEG_Y0 + LEG_Y1) / 2, seg=24)
    return solid - cuts


def make_sector():
    pts, _ = gear_xs(Z_RACK)
    sec = ext(CrossSection([pts]) ^ sector_xs(SEC_R0, R_RACK + 2 * MOD, -SEC_HALF, SEC_HALF),
              Z_FLOOR, Z_FLOOR + SECTOR_T)
    zt = Z_FLOOR + SECTOR_T
    cuts = cube(TAPE_R[0], TAPE_R[1], -TAPE_W / 2, TAPE_W / 2, zt - TAPE_D, zt + 1)   # canal de la cinta
    tri = CrossSection([[(SEC_R0 - 0.5, -2.0), (SEC_R0 + 4.0, 0.0), (SEC_R0 - 0.5, 2.0)]])
    cuts += ext(tri, zt - 0.6, zt + 1)                                                  # índice
    return sec - cuts


def make_pinion():
    """Piñón en su posición: hueco del horn en la cara de arriba, avellanado abajo."""
    g = gear_full(Z_PIN, 0, PIN_T, phase=180.0 / Z_PIN)
    g -= s.horn_cuts(0.0)                      # hueco abajo, avellanado arriba (como el brazo)
    g = g.rotate((180, 0, 0)).translate((0, 0, PIN_T))   # hueco arriba (hacia el servo)
    return g.translate((X_PIN, 0, PIN_Z0))


def servo2_ghost():
    """Servo perturbador boca abajo; eje en (X_PIN, 0), cuerpo hacia +X."""
    g = s.servo_ghost().translate((0, 0, -s.PLATE_T))       # fondo del servo en z = 0
    g = g.rotate((0, 180, 0))                                # boca abajo; el cuerpo queda hacia +X
    return g.translate((X_PIN, 0, Z_BOSS2 + s.SV_TOP))


def horn2_ghost():
    h = s.horn_ghost().translate((0, 0, -(s.PLATE_T + s.SV_HORN_SEAT)))   # cubo desde z=0 hacia arriba
    h = h.rotate((180, 0, 0))                                             # paleta abajo
    return h.translate((X_PIN, 0, HORN_SEAT2))


BR_HOLES_X = (BR_X0 + 6.0, BR_X1 - 6.0)


def make_bracket():
    """Puente sobre el piñón; las orejas del servo apoyan arriba. Ranuras de 3 mm para desengranar."""
    xe0 = X_PIN - s.SV_SHAFT_END                         # cuerpo hacia +X desde el eje
    xe1 = xe0 + s.SV_L
    xc = (xe0 + xe1) / 2
    z0, z1 = Z_TAB2 - BR_PLATE_T, Z_TAB2
    plate = cube(BR_X0, BR_X1, -LEG_Y1, LEG_Y1, z0, z1)
    legs = cube(BR_X0, BR_X1, LEG_Y0, LEG_Y1, Z_SURF, z1) + cube(BR_X0, BR_X1, -LEG_Y1, -LEG_Y0, Z_SURF, z1)
    solid = plate + legs
    cuts = cube(xe0 - s.FIT_X, xe1 + s.FIT_X, -s.SV_W / 2 - s.FIT_Y, s.SV_W / 2 + s.FIT_Y, z0 - 1, z1 + 1)
    for sx in (-1, 1):
        cuts += cyl(s.SV_PILOT_D / 2, z0 - 1, z1 + 1, x=xc + sx * s.SV_HOLE_SP / 2, seg=24)
    for x in BR_HOLES_X:                                 # ranuras radiales M3 (3 mm de recorrido)
        for sy in (-1, 1):
            y = sy * (LEG_Y0 + LEG_Y1) / 2
            slot = Manifold.hull(Manifold.compose([cyl(1.75, Z_SURF - 1, z1 + 1, x=x - 1.0, y=y, seg=24),
                                                   cyl(1.75, Z_SURF - 1, z1 + 1, x=x + 2.0, y=y, seg=24)]))
            cuts += slot
            cuts += Manifold.hull(Manifold.compose([cyl(3.2, Z_SURF + 3.0, z1 + 1, x=x - 1.0, y=y, seg=32),
                                                    cyl(3.2, Z_SURF + 3.0, z1 + 1, x=x + 2.0, y=y, seg=32)]))
    return solid - cuts


# =====================================================================
#  Ensamble y verificación
# =====================================================================
def ensamble(theta=0.0, arm_ang=0.0):
    dz = Z_SURF
    p = dict(
        pedestal=make_pedestal(), pista=make_track(), soporte=make_bracket(),
        sector=rot(make_sector(), theta),
        pinon=rot(make_pinion().translate((-X_PIN, 0, 0)), -theta * RATIO).translate((X_PIN, 0, 0)),
        servo2=servo2_ghost(), horn2=horn2_ghost(),
        base=s.make_base(R_FOOT).translate((0, 0, dz)),
        servo=s.servo_ghost().translate((0, 0, dz)), horn=s.horn_ghost().translate((0, 0, dz)),
        brazo=rot(s.make_arm(L), arm_ang).translate((0, 0, dz)),
    )
    return p


def verificar():
    res = {}
    ped, tr, br, sec0, pin0 = make_pedestal(), make_track(), make_bracket(), make_sector(), make_pinion()
    sv2, hn2 = servo2_ghost(), horn2_ghost()
    base = s.make_base(R_FOOT).translate((0, 0, Z_SURF))
    res["base∩pedestal"] = vol(base ^ ped)
    res["pedestal∩pista"] = vol(ped ^ tr)
    res["servo2∩soporte"] = vol(sv2 ^ br)
    res["servo2∩pista"] = vol(sv2 ^ tr)
    res["horn2∩piñón"] = vol(hn2 ^ pin0)
    res["piñón∩soporte/pista"] = vol(pin0 ^ (br + tr))
    res["soporte∩pista"] = vol(br ^ tr)
    worst_sec, worst_gear = 0.0, 0.0
    for th in np.linspace(-SEC_TRAVEL, SEC_TRAVEL, 11):
        sec = rot(sec0, th)
        pin = rot(pin0.translate((-X_PIN, 0, 0)), -th * RATIO).translate((X_PIN, 0, 0))
        worst_sec = max(worst_sec, vol(sec ^ (tr + ped + br)))
        worst_gear = max(worst_gear, vol(sec ^ pin))
    res["sector(±12,5°)∩pista/pedestal/soporte"] = worst_sec
    res["sector∩piñón engranando (mm³)"] = worst_gear
    worst = 0.0
    fixed = ped + tr + br + sv2 + pin0 + rot(sec0, SEC_TRAVEL) + rot(sec0, -SEC_TRAVEL)
    for a in range(-30, 31, 5):
        arm = rot(s.make_arm(L), a).translate((0, 0, Z_SURF))
        worst = max(worst, vol(arm ^ fixed))
    res["brazo(±30°)∩plataforma/servo2"] = worst
    # holgura vertical LDR -> superficie de la línea
    res["boca de la cabina LDR -> superficie del sector (mm)"] = round(s.FLOOR_GAP + Z_SURF - (Z_FLOOR + SECTOR_T), 2)
    return res


PIEZAS = {
    "P1_pedestal.stl": lambda: make_pedestal(),
    "P2_pista.stl": lambda: make_track(),
    "P3_sector_linea.stl": lambda: make_sector().translate((0, 0, -Z_FLOOR)),
    "P4_pinon.stl": lambda: make_pinion().translate((-X_PIN, 0, -PIN_Z0)),
    "P5_soporte_perturbador.stl": lambda: s.print_orientation(make_bracket()).translate((-BR_X0, 0, 0)),
}


def main():
    s.OUT = os.path.abspath(OUT)
    for n, f in PIEZAS.items():
        s.save(f(), n)
    print(f"Engranaje: módulo {MOD}, cremallera {Z_RACK} (R={R_RACK}), piñón {Z_PIN} (R={R_PIN}), "
          f"relación {RATIO:.3f}:1 -> 1° de servo = {1 / RATIO:.3f}° de línea; 180° = {180 / RATIO:.1f}°")
    print(f"Superficie de la línea z={Z_SURF}; servo perturbador: tapa z={Z_BOSS2:.2f}, orejas z={Z_TAB2:.2f}, "
          f"fondo z={Z_BOSS2 + s.SV_TOP:.2f}")
    for k, v in verificar().items():
        print(f"   {k:42s} {round(v, 3) if isinstance(v, float) else v}")


if __name__ == "__main__":
    main()
