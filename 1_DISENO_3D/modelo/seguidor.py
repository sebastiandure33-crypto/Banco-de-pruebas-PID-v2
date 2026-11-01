#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Seguidor de línea estático con servo SG90.
Genera: base del servo (con torre guía de cables), brazo (alojamiento del horn, canal de
cables y cabezal LDR/LED).

Uso:    python3 seguidor.py [L1 L2 ...]      (L = distancia eje servo -> centro de los LDR, en mm)
Requiere: pip install manifold3d trimesh numpy
Unidades: mm. Origen: eje del servo en (0,0). z=0 es el fondo de la base.
El brazo apunta a +X (servo en 90°); el barrido de 60°–120° es +-30° alrededor de +X.

Recorrido de cables: cabezal -> canal sobre el brazo -> salen hacia arriba junto al eje
(r ≈ 6 mm) -> anillo guía fijo encima del eje -> bajan por la torre (precintos) -> placa.
Como salen casi sobre el eje, el giro de ±30° apenas los desplaza y no carga el brazo.
"""
import os
import sys
import math
import numpy as np
import trimesh
from manifold3d import Manifold

# =====================================================================
#  PARÁMETROS  (medir con calibre y ajustar si hace falta)
# =====================================================================
# --- Servo SG90 ---
SV_L, SV_W = 22.5, 12.2          # cuerpo sin orejas (plano: 22,5 x 11,8; algunas marcas 12,15)
SV_BODY_H = 22.7                 # altura del cuerpo (sin la parte redonda del eje)
SV_TAB_L = 31.9                  # largo total con orejas (4,7 + 22,5 + 4,7)
SV_TAB_T = 2.5                   # espesor de la oreja
SV_TAB_Z = 15.9                  # cara inferior de la oreja, medida desde el fondo del servo
SV_SHAFT_END = 5.9               # distancia del eje al extremo del cuerpo (lado del eje)
SV_HOLE_SP = 27.3                # distancia entre agujeros de las orejas (31,9 - 2 x 2,3)
SG90_STALL = 137.3               # N·mm: par de bloqueo 1,4 kg·cm a 4,8 V (1,9 a 6 V)
SV_PILOT_D = 1.7                 # agujero guía para tornillo autorroscante M2
SV_TOP = 26.7                    # tapa superior del servo (22,7 + 4,0), sin el eje
SV_HORN_SEAT = 27.0              # cara inferior del horn colocado (≈0,3 sobre la tapa)

# --- Horn (el de un solo brazo que trae el SG90) ---
HORN_HUB_D = 7.2                 # diámetro del cubo
HORN_HUB_H = 4.0                 # altura del cubo
HORN_PAD_L = 16.4                # del eje al extremo de la paleta (horn de 19,6 mm total)
HORN_PAD_W0 = 7.2                # ancho de la paleta junto al cubo
HORN_PAD_W1 = 3.8                # ancho de la paleta en la punta
HORN_PAD_T = 1.6                 # espesor de la paleta
HORN_CLR = 0.10                  # holgura por lado del hueco del horn
HORN_GAP = 0.5                   # separación entre el brazo y el asiento del horn

# --- Base ---
PLATE_T = 3.0
FIT_X, FIT_Y = 0.20, 0.15        # holgura del bolsillo del servo
WALL_Y = 2.1                     # espesor de pared lateral
MOUNT_D = 3.4                    # agujeros de fijación M3
CSLOT_Y0, CSLOT_Y1 = -9.5, -2.2  # ranura abierta por arriba para el cable del servo (lado -Y):
                                 # el cable ya soldado a la placa se acuesta desde arriba
FOOT_ANG = 52.0                  # semiángulo del pie en abanico (°)
STAB_MARGIN = 8.0                # margen de estabilidad pedido (mm)

# --- Brazo ---
# El horn va con el cubo estriado HACIA ABAJO (hacia el servo) y la paleta plana ARRIBA, al ras
# de la cara superior del cubo. El brazo apoya sobre esa cara: la paleta y la parte alta del cubo
# entran en un hueco de la cara inferior del brazo; el resto del cubo queda debajo, libre.
POCKET_D = HORN_PAD_T - HORN_GAP + 0.15   # profundidad del hueco del horn (paleta + cubo)
ROOF_T = 1.5                     # material entre el horn y la cabeza del tornillo
SCREW_HEAD_D = 4.4               # avellanado para la cabeza del tornillo de 2 x 6 mm
ARM_T = 5.05                     # espesor del brazo (cara superior toda plana)
HUB_DEPTH = POCKET_D             # (compatibilidad)
ARM_W = 9.6                      # ancho (lleva el canal de cables)
HUB_R = 6.5                      # radio del cubo del brazo
SCREW_D = 2.4                    # paso del tornillo central del horn

# --- Cables ---
N_CABLES = 10
CABLE_D = 1.0                    # diámetro exterior de cada cable (cable de red ≈ 0.9–1.0)
CABLE_G_MM = 2.24e-3             # g/mm por cable (cobre 24 AWG + aislación)
GROOVE_W = 6.8                   # canal: 6 cables abajo + 4 arriba (entre las pestañas)
GROOVE_D = 2.4                   # 2 capas
GROOVE_X0 = 5.8                  # inicio del canal = punto de salida de los cables (r al eje)
LIP = 1.1                        # pestañas que retienen los cables en el canal
LIP_L = 4.0
LIP_PITCH = 18.0
XGROOVE_W = 4.0                  # canal transversal del cabezal

# --- Torre guía de cables (fija, parte de la base) ---
TOWER_FREE = 65.0                # cable libre entre el brazo y el anillo (más = menos esfuerzo)
RING_ID = 6.5
RING_OD = 13.0
RING_H = 6.0
RING_GAP = 1.4                   # abertura para meter los cables a presión sin desoldar
POST_W = 5.5                     # sección de la torre (y)
TIE_W, TIE_H = 1.6, 3.2          # ranuras para precintos

# --- Cabezal ---
LDR_SP = 20.0                    # distancia entre centros de los 2 LDR (≈ ancho de la línea)
FLOOR_GAP = 7.0                  # ventana del LDR -> piso (base apoyada en el piso)
COL_R = 4.1                      # radio exterior de la cabina del LDR
LDR_BORE_D = 5.5                 # LDR de 5 mm (cuerpo ≈ 5.1)
LDR_WIN_D = 3.6                  # ventana de entrada de luz
LDR_LEDGE = 1.0                  # espesor del tope
LEG_SLOT_W = 4.8                 # ranura de salida de patitas (hacia el brazo)
LEG_SLOT_H = 5.0
LED_DX = 7.5                     # LED a +X del LDR (a nivel de la ventana)
LED_TILT = 35.0                  # inclinación del LED hacia el LDR (grados desde la vertical)
LED_R = 4.5                      # radio exterior del tubo del LED
LED_BORE_D = 5.2                 # LED de 5 mm
LED_REC_D = 6.3                  # alojamiento del reborde del LED (≈5.8)
LED_STEP = 8.0                   # del plano de salida al reborde
LED_LEN = 11.3                   # largo del tubo
ROOF_ANG = 50.0                  # techo del tubo del LED (° sobre la horizontal): >45° = sin soportes
ROOF_Z = 27.5                    # altura del techo junto a la cabina del LDR

ZA = PLATE_T + SV_HORN_SEAT + (HORN_HUB_H - HORN_PAD_T) + HORN_GAP   # cara inferior del brazo
ZT = ZA + ARM_T                            # cara superior del brazo
ZR0 = ZT + TOWER_FREE                      # cara inferior del anillo guía

SEG = 64
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "variantes")   # 1_DISENO_3D/variantes


# =====================================================================
#  Utilidades
# =====================================================================
def cube(x0, x1, y0, y1, z0, z1):
    return Manifold.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0, seg=SEG):
    return Manifold.cylinder(z1 - z0, r, r, seg).translate((x, y, z0))


def hull(*ms):
    return Manifold.compose(list(ms)).hull()


def tilt(m, x, y, z):
    return m.rotate((0.0, LED_TILT, 0.0)).translate((x, y, z))


def vol(m):
    try:
        return m.volume()
    except AttributeError:
        return m.get_volume()


def to_tm(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3].astype(float),
                           faces=np.asarray(mesh.tri_verts), process=False)


def save(m, name):
    os.makedirs(OUT, exist_ok=True)
    tm = to_tm(m)
    path = os.path.join(OUT, name)
    tm.export(path)
    ok = trimesh.load(path).is_watertight          # se comprueba el archivo tal como lo leerá el slicer
    print(f"{name:28s} watertight={ok}  vol={tm.volume/1000:.2f} cm3  "
          f"bbox={np.round(tm.extents, 1).tolist()}")
    return tm


def print_orientation(m):
    """Gira 180° para imprimir con la cara plana superior sobre la cama."""
    m = m.rotate((180.0, 0.0, 0.0))
    zmin = m.bounding_box()[2]
    return m.translate((0, 0, -zmin))


# =====================================================================
#  Piezas
# =====================================================================
def servo_ghost():
    xe0, xe1 = SV_SHAFT_END - SV_L, SV_SHAFT_END
    xc = (xe0 + xe1) / 2
    body = cube(xe0, xe1, -SV_W / 2, SV_W / 2, PLATE_T, PLATE_T + SV_BODY_H)
    tabs = cube(xc - SV_TAB_L / 2, xc + SV_TAB_L / 2, -SV_W / 2, SV_W / 2,
                PLATE_T + SV_TAB_Z, PLATE_T + SV_TAB_Z + SV_TAB_T)
    boss = cyl(5.9, PLATE_T + SV_BODY_H, PLATE_T + SV_TOP)                     # tapa redonda Ø11,8
    boss += cube(-8.8, 0, -2.5, 2.5, PLATE_T + SV_BODY_H, PLATE_T + SV_TOP)    # lóbulo de 5 mm
    boss += cyl(2.3, PLATE_T + SV_TOP, PLATE_T + SV_TOP + 3.2)                 # eje estriado Ø4,6
    return body + tabs + boss


def horn_ghost():
    zh = PLATE_T + SV_HORN_SEAT
    zp = zh + HORN_HUB_H - HORN_PAD_T                 # la paleta está ARRIBA, al ras del cubo
    hub = cyl(HORN_HUB_D / 2, zh, zh + HORN_HUB_H)    # el cubo baja hasta el eje del servo
    pad = hull(cyl(HORN_PAD_W0 / 2, zp, zh + HORN_HUB_H),
               cyl(HORN_PAD_W1 / 2, zp, zh + HORN_HUB_H, x=HORN_PAD_L - HORN_PAD_W1 / 2))
    return hub + pad


def screw_ghost():
    """Cabeza del tornillo del horn + destornillador (debe poder pasar por el anillo)."""
    return cyl(2.3, ZT, ZT + 2.0) + cyl(1.6, ZT + 2.0, ZR0 + RING_H + 20)


def wedge(r, half_ang, z0, z1):
    """Sector circular |θ| <= half_ang, centrado en el eje del servo."""
    sec = cyl(r, z0, z1, seg=128)
    up = cube(-2 * r, 2 * r, 0, 2 * r, z0 - 1, z1 + 1).rotate((0, 0, -half_ang))
    dn = cube(-2 * r, 2 * r, -2 * r, 0, z0 - 1, z1 + 1).rotate((0, 0, half_ang))
    return sec ^ up ^ dn


def base_dims():
    xe0, xe1 = SV_SHAFT_END - SV_L, SV_SHAFT_END
    xc = (xe0 + xe1) / 2
    wx0, wx1 = xc - SV_TAB_L / 2 - 0.3, xc + SV_TAB_L / 2 + 0.3
    wy = SV_W / 2 + FIT_Y + WALL_Y
    return xe0, xe1, xc, wx0, wx1, wy


def make_tower():
    """Torre lateral-trasera + brazo a 45° (sin soportes) + anillo guía centrado en el eje."""
    _, _, _, wx0, _, wy = base_dims()
    px0, px1 = wx0 - 7.5, wx0 - 0.5            # x de la torre (detrás, sin tapar tornillos)
    py0, py1 = wy, wy + POST_W
    zp = ZR0 - 26.0                            # techo de la torre -> voladizo >= 45°
    solid = cube(wx0 - 8.0, wx0 - 4.0, py0 - 1.0, py1, 0, PLATE_T)    # ala de la placa
    solid += cube(px0, px1, py0, py1, 0, zp)
    solid += hull(cube(px0, px1, py0, py1, zp - 6.0, zp), cyl(RING_OD / 2, ZR0, ZR0 + RING_H))
    cuts = cyl(RING_ID / 2, ZR0 - 40, ZR0 + RING_H + 1)
    cuts += Manifold.cylinder(1.6, RING_ID / 2, RING_ID / 2 + 1.6, SEG).translate((0, 0, ZR0 + RING_H - 1.6 + 0.01))
    cuts += cube(-RING_GAP / 2, RING_GAP / 2, -RING_OD, 0, ZR0 - 40, ZR0 + RING_H + 1)   # abertura (-Y)
    for zt in (40.0, 62.0):                                                              # precintos
        cuts += cube(px0 + 1.4, px0 + 1.4 + TIE_W, py0 - 1, py1 + 1, zt, zt + TIE_H)
    return solid - cuts


def make_base(r_foot=0.0, tower=True):
    """r_foot = radio del pie en abanico antivuelco (0 = base compacta, para atornillar)."""
    xe0, xe1, xc, wx0, wx1, wy = base_dims()
    px0, px1 = xe0 - FIT_X, xe1 + FIT_X
    py = SV_W / 2 + FIT_Y
    z_tab_u = PLATE_T + SV_TAB_Z
    z_tab_t = z_tab_u + SV_TAB_T

    solid = cube(wx0 - 4, wx1 + 4, -(wy + 5.5), wy + 5.5, 0, PLATE_T)
    if r_foot > 0:                                       # pie en abanico (pasa bajo el brazo)
        solid += wedge(r_foot, FOOT_ANG, 0, PLATE_T)
    for s in (-1, 1):                                   # paredes laterales
        y0, y1 = sorted((s * py, s * wy))
        solid += cube(wx0, wx1, y0, y1, 0, z_tab_t)
    solid += cube(wx0, px0, -wy, wy, 0, z_tab_u)         # pilares de las orejas
    solid += cube(px1, wx1, -wy, wy, 0, z_tab_u)
    if tower:
        solid += make_tower()

    cuts = cube(px0, px1, -py, py, PLATE_T, z_tab_t + 5)  # bolsillo del servo
    cuts += cube(wx0 - 9, px0 + 0.1, -4.5, 4.5, PLATE_T, PLATE_T + 6)   # salida del cable del servo
    cuts += cube(wx0 - 9, px0 + 0.1, CSLOT_Y0, CSLOT_Y1, PLATE_T, z_tab_t + 5)  # ranura abierta arriba
    for sx in (-1, 1):
        xh = xc + sx * SV_HOLE_SP / 2
        cuts += cyl(SV_PILOT_D / 2, z_tab_u - 10, z_tab_u + 1, x=xh, seg=24)
        for sy in (-1, 1):
            cuts += cyl(MOUNT_D / 2, -1, PLATE_T + 1, x=xh, y=sy * (wy + 2.75), seg=32)
    if r_foot > 0:                                       # fijación opcional en el pie
        for sa in (-1, 0, 1):
            a = math.radians(sa * (FOOT_ANG - 9))
            rr = r_foot - 5.5
            cuts += cyl(MOUNT_D / 2, -1, PLATE_T + 1,
                        x=rr * math.cos(a), y=rr * math.sin(a), seg=32)
    return solid - cuts


def horn_cuts(za):
    """Hueco con la forma de la cara superior del horn (cubo + paleta = antigiro),
    paso del tornillo y avellanado para su cabeza."""
    c = HORN_CLR
    cut = cyl(HORN_HUB_D / 2 + c, za - 1, za + POCKET_D)
    cut += hull(cyl(HORN_PAD_W0 / 2 + c, za - 1, za + POCKET_D),
                cyl(HORN_PAD_W1 / 2 + c, za - 1, za + POCKET_D, x=HORN_PAD_L - HORN_PAD_W1 / 2))
    cut += cyl(SCREW_D / 2, za + POCKET_D - 0.5, za + ARM_T + 1, seg=32)
    cut += cyl(SCREW_HEAD_D / 2, za + POCKET_D + ROOF_T, za + ARM_T + 1, seg=48)
    return cut


def make_arm(L):
    za, zt = ZA, ZT
    zb = FLOOR_GAP                             # plano de las ventanas de los sensores
    yc = LDR_SP / 2
    xb0 = L - COL_R - 5.0                      # travesaño del cabezal (lleva el canal transversal)

    solid = cyl(HUB_R, za, zt)
    solid += hull(cyl(HUB_R, za, zt), cube(HUB_R + 6, HUB_R + 6.5, -ARM_W / 2, ARM_W / 2, za, zt))
    solid += cube(HUB_R + 6, L, -ARM_W / 2, ARM_W / 2, za, zt)
    solid += cube(xb0, L + COL_R, -yc, yc, za, zt)

    cuts = horn_cuts(za)
    # canal longitudinal (abierto hacia arriba) y canal transversal del cabezal
    xg0, xg1 = xb0 + 1.6, xb0 + 1.6 + XGROOVE_W
    cuts += cube(GROOVE_X0, xg1, -GROOVE_W / 2, GROOVE_W / 2, zt - GROOVE_D, zt + 1)
    cuts += cube(xg0, xg1, -yc, yc, zt - GROOVE_D, zt + 1)
    for s in (-1, 1):
        y = s * yc
        solid += cyl(COL_R, zb, zt, x=L, y=y)                      # cabina del LDR
        # tubo del LED unido a la cabina, con techo inclinado (imprimible boca abajo sin soportes)
        led_out = tilt(cyl(LED_R, -4, LED_LEN + 14, seg=48), L + LED_DX, y, zb)
        roof = cube(-300, 300, -300, 300, -600, 0).rotate((0, ROOF_ANG, 0)).translate((L + COL_R, 0, ROOF_Z))
        solid += hull(cyl(COL_R, zb, zt, x=L, y=y), led_out) ^ roof ^ cube(L, L + 80, y - 20, y + 20, zb - 5, zt)

        cuts += cyl(LDR_WIN_D / 2, zb - 1, zb + LDR_LEDGE + 0.2, x=L, y=y, seg=48)      # ventana
        cuts += cyl(LDR_BORE_D / 2, zb + LDR_LEDGE, zt + 1, x=L, y=y, seg=48)           # alojamiento
        cuts += cube(xg0, L, y - LEG_SLOT_W / 2, y + LEG_SLOT_W / 2,
                     zt - LEG_SLOT_H, zt + 1)                                           # salida patitas
        cuts += cube(L, L + COL_R + 1, y - 1.3, y + 1.3, zt - 3.0, zt + 1)              # entrada cables LED
        cuts += tilt(cyl(LED_BORE_D / 2, -5, LED_STEP + 0.5, seg=48), L + LED_DX, y, zb)  # LED
        cuts += tilt(cyl(LED_REC_D / 2, LED_STEP, LED_LEN + 30, seg=48), L + LED_DX, y, zb)

    solid = solid ^ cube(-50, L + 60, -60, 60, zb, zt + 50)       # plano inferior de los sensores
    arm = solid - cuts
    # pestañas que retienen los cables (quedan sobre la cama al imprimir: sin soportes)
    lips = None
    x = GROOVE_X0 + 14.0
    while x + LIP_L < xg0 - 4.0:
        for s in (-1, 1):
            y0, y1 = sorted((s * GROOVE_W / 2, s * (GROOVE_W / 2 - LIP)))
            p = cube(x, x + LIP_L, y0, y1, zt - 1.0, zt)
            lips = p if lips is None else lips + p
        x += LIP_PITCH
    return arm + lips if lips is not None else arm


def make_horn_test():
    clrs = (HORN_CLR - 0.05, HORN_CLR, HORN_CLR + 0.10)   # 1, 2 y 3 puntos; 2 puntos = la del brazo
    h = POCKET_D + 1.6
    solid = None
    cuts = None
    for i, c in enumerate(clrs):
        y0 = i * 17.0
        blk = cube(-9, 20, y0 - 8.6, y0 + 8.6, 0, h)            # bloques unidos: una sola pieza
        solid = blk if solid is None else solid + blk
        k = cyl(HORN_HUB_D / 2 + c, -1, POCKET_D, y=y0)
        k += hull(cyl(HORN_PAD_W0 / 2 + c, -1, POCKET_D, y=y0),
                  cyl(HORN_PAD_W1 / 2 + c, -1, POCKET_D, x=HORN_PAD_L - HORN_PAD_W1 / 2, y=y0))
        for j in range(i + 1):                         # marcas: 1, 2 o 3 puntos
            k += cyl(0.6, -1, 0.8, x=17.8, y=y0 - 2.4 * i + 2.4 * j, seg=16)
        cuts = k if cuts is None else cuts + k
    return solid - cuts


# =====================================================================
#  Masas, estabilidad y verificación
# =====================================================================
M_SERVO = 9.0      # g, SG90 con su cable
RHO = 1.24e-3      # g/mm³, PLA


def masa_cm(m):
    t = to_tm(m)
    t.density = RHO
    return t.mass, t.center_mass[0], t.center_mass[1], t.bounds[1][0], t.bounds[1][1], t


def arm_props(L):
    """Masa, CoM e inercia (respecto al eje) del brazo + los cables que lleva encima."""
    m, cx, cy, _, _, t = masa_cm(make_arm(L))
    I = t.moment_inertia[2, 2] + m * (cx ** 2 + cy ** 2)
    lw = L + LDR_SP / 2 - GROOVE_X0                         # largo de cable sobre el brazo
    mw = N_CABLES * CABLE_G_MM * lw
    xw = GROOVE_X0 + (L - GROOVE_X0) / 2
    Iw = N_CABLES * CABLE_G_MM * ((L ** 3 - GROOVE_X0 ** 3) / 3 + LDR_SP / 2 * L ** 2)
    mt = m + mw
    return mt, (m * cx + mw * xw) / mt, m * cy / mt, I + Iw, m, mw


def margen_vuelco(L, r_foot, tower=True):
    """Distancia del CoM del conjunto al borde de apoyo, en el peor ángulo. <0 = vuelca."""
    mb, bx, by, bxe, bye, _ = masa_cm(make_base(r_foot, tower))
    ma, ax, ay, _, _, _ = arm_props(L)
    xs = SV_SHAFT_END - SV_L / 2
    mt = mb + ma + M_SERVO
    peor = None
    for ang in range(-30, 31, 5):
        r = math.radians(ang)
        xr = ax * math.cos(r) - ay * math.sin(r)
        yr = ax * math.sin(r) + ay * math.cos(r)
        gx = (mb * bx + M_SERVO * xs + ma * xr) / mt
        gy = (mb * by + ma * yr) / mt
        d_plate = min(bxe - gx, 13.85 - abs(gy))          # placa rectangular del servo
        if r_foot > 0 and abs(math.degrees(math.atan2(gy, gx))) <= FOOT_ANG - 4:
            d = max(d_plate, r_foot - math.hypot(gx, gy))
        else:
            d = d_plate
        if peor is None or d < peor[0]:
            peor = (d, ang, mt)
    return peor


def radio_pie(L):
    for r in range(20, 130, 2):
        if margen_vuelco(L, float(r))[0] >= STAB_MARGIN:
            return float(r)
    return 130.0


def verificar(arm, base, L):
    sv, hg, sc = servo_ghost(), horn_ghost(), screw_ghost()
    res = {"servo∩base": vol(sv ^ base), "horn∩brazo": vol(hg ^ arm),
           "tornillo/destornillador∩torre": vol(sc ^ base)}
    worst = 0.0
    for ang in range(-30, 31, 5):
        a = arm.rotate((0, 0, ang))
        worst = max(worst, vol(a ^ base), vol(a ^ sv))
    res["brazo(±30°)∩base/torre/servo"] = worst
    yc = LDR_SP / 2
    ldr = cyl(LDR_BORE_D / 2 + 0.6, FLOOR_GAP, ZT, x=L, y=yc, seg=48)
    led = tilt(cyl(LED_BORE_D / 2 + 0.6, -5, LED_STEP, seg=48), L + LED_DX, yc, FLOOR_GAP)
    res["LDR∩LED (con 0.6 de pared)"] = vol(ldr ^ led)
    res["10 cables∩brazo (en el canal)"] = vol(cables_en_canal(L) ^ arm)
    # asiento del reborde del LED: debe quedar rodeado por >= 1 mm de material
    for sd in (-1, 1):
        ring = tilt(cyl(LED_REC_D / 2 + 1.0, LED_STEP - 1.0, LED_STEP, seg=48) -
                    cyl(LED_BORE_D / 2, LED_STEP - 2, LED_STEP + 1, seg=48), L + LED_DX, sd * yc, FLOOR_GAP)
        res[f"asiento LED {'+' if sd > 0 else '-'} sin material (mm³)"] = vol(ring - arm)
    res["voladizo sin apoyo al imprimir (mm²)"] = voladizo_libre(print_orientation(arm))
    # impresión: toda la cara superior debe apoyar en la cama
    t = to_tm(print_orientation(arm))
    zc = t.triangles_center[:, 2]
    bed = (zc < 1e-3) & (t.face_normals[:, 2] < -0.99)
    xs = t.triangles_center[bed][:, 0]
    res["apoyo en cama de x a x (mm)"] = (round(float(xs.min()), 1), round(float(xs.max()), 1))
    return res


def voladizo_libre(m, ang=45.0, max_puente=8.0):
    """Área (mm²) de caras que miran hacia abajo con más de `ang`° de voladizo y que NO son
    puentes: un puente tiene paredes a ambos lados (en X o en Y) separadas <= max_puente."""
    t = to_tm(m)
    nz = t.face_normals[:, 2]
    c = t.triangles_center
    sel = (nz < -math.cos(math.radians(90 - ang))) & (c[:, 2] > 0.3)
    libre = 0.0
    for p, a in zip(c[sel], t.area_faces[sel]):
        o = p - np.array([0, 0, 0.2])
        apoyado = False
        for ax in (np.array([1.0, 0, 0]), np.array([0, 1.0, 0])):
            locs, ri, _ = t.ray.intersects_location([o, o], [ax, -ax])
            if len(ri) and set(ri.tolist()) == {0, 1}:
                d = [np.min(np.linalg.norm(locs[ri == k] - o, axis=1)) for k in (0, 1)]
                if d[0] + d[1] <= max_puente:
                    apoyado = True
                    break
        if not apoyado:
            libre += a
    return round(float(libre), 1)


def posiciones_cables():
    """(y, z) de cada cable dentro del canal: 6 abajo + 4 arriba."""
    r = CABLE_D / 2
    z0 = ZT - GROOVE_D + r
    abajo = [(-2.75 + 1.1 * i, z0) for i in range(6)]
    arriba = [(-1.65 + 1.1 * i, z0 + CABLE_D) for i in range(4)]
    return abajo + arriba


def cables_en_canal(L):
    """Los 10 cables como cilindros a lo largo del canal (para verificar que entran)."""
    r = CABLE_D / 2 - 0.02
    m = None
    for y, z in posiciones_cables():
        c = Manifold.cylinder(L - GROOVE_X0 - 14, r, r, 16).rotate((0, 90, 0)).translate((GROOVE_X0 + 2, y, z))
        m = c if m is None else m + c
    return m


def esfuerzo_cables(L, ang=30.0):
    """Par que los cables le hacen al servo y deformación del cobre en el tramo libre."""
    th = math.radians(ang)
    r = GROOVE_X0 + 0.5
    l = TOWER_FREE
    delta = r * th
    out = {}
    for nombre, d_cu, n_hilos in (("rígido (red)", 0.51, 1), ("multifilar 7x0.2", 0.20, 7)):
        E, G = 117e3, 44e3
        I1 = n_hilos * math.pi * d_cu ** 4 / 64
        J1 = 2 * I1
        F = 12 * E * I1 * delta / l ** 3                     # extremos empotrados (peor caso)
        T_flex = N_CABLES * F * r
        T_tors = N_CABLES * G * J1 * th / (l + 40.0)         # torsión hasta el 1er precinto
        eps = 3 * delta * d_cu / l ** 2                      # deformación máx. por flexión
        out[nombre] = (T_flex + T_tors, eps)
    return out


def tabla(longitudes, ang=30.0):
    stall = SG90_STALL
    print("\n  L(mm) | cobertura ±mm | mm por grado | masa brazo+cables g | I (kg·m²) | acel. (rad/s²)")
    for L in longitudes:
        m, cx, cy, I, _, _ = arm_props(L)
        I = I * 1e-9
        a = 0.25 * stall * 1e-3 / I
        print(f"  {L:5.0f} | {L*math.sin(math.radians(ang)):13.1f} | {L*math.pi/180:12.2f} | "
              f"{m:19.1f} | {I:.2e} | {a:9.0f}")


def main():
    Ls = [float(a) for a in sys.argv[1:]] or [100.0, 120.0, 150.0]
    assert LDR_SP >= 12.0, "LDR_SP demasiado chico para las cabinas"
    base = make_base()
    save(base, "base_compacta_atornillar.stl")
    d0, a0, mt0 = margen_vuelco(Ls[0], 0.0)
    print(f"   base compacta: margen de vuelco {d0:+.1f} mm -> {'ATORNILLAR obligatorio' if d0 < 0 else 'se sostiene sola'}")
    for L in Ls:
        arm = make_arm(L)
        save(print_orientation(arm), f"brazo_L{int(L)}.stl")
        print("   verificación:", {k: (round(v, 3) if isinstance(v, float) else v)
                                    for k, v in verificar(arm, base, L).items()})
        e = radio_pie(L)
        bext = make_base(e)
        save(bext, f"base_apoyo_libre_L{int(L)}.stl")
        d, a, mt = margen_vuelco(L, e)
        print(f"   pie r={e:.0f} mm -> margen {d:+.1f} mm a {a:+d}°, conjunto {mt:.1f} g")
        print("   verificación:", {k: (round(v, 3) if isinstance(v, float) else v)
                                    for k, v in verificar(arm, bext, L).items()})
    print(f"\nCara superior del brazo z={ZT:.2f}; anillo guía z={ZR0:.1f}–{ZR0+RING_H:.1f}; cable libre {TOWER_FREE:.0f} mm")
    print(f"Par de los cables sobre el servo a ±30° (SG90 = {SG90_STALL:.0f} N·mm a 4,8 V):")
    for k, (T, eps) in esfuerzo_cables(Ls[0]).items():
        print(f"   {k:18s}: {T:5.2f} N·mm = {T/SG90_STALL*100:4.1f} %   deformación cobre {eps*100:.3f} % "
              f"({'elástico' if eps < 0.0006 else 'cerca/sobre fluencia'})")
    tabla([80, 100, 120, 150, 180])


if __name__ == "__main__":
    main()
