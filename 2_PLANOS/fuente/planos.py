"""Planos del seguidor: genera PDF A3 multipágina + PNG por hoja."""
import sys, math
import os
_AQUI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(_AQUI, "..", "..", "1_DISENO_3D", "modelo"))
sys.path.insert(0, _AQUI)
import numpy as np
import seguidor as s
from manifold3d import Manifold
from lib_plano import *
from matplotlib.patches import Arc
from matplotlib.backends.backend_pdf import PdfPages

OUT = os.path.join(_AQUI, "..") + os.sep          # 2_PLANOS/
L = 120.0
TOTAL = 5
GRIS = "#d9d9d9"
GRIS2 = "#ececec"

ARM = s.make_arm(L)
BASE = s.make_base(34.0)
SERVO = s.servo_ghost()
HORN = s.horn_ghost()


# ------------------------------------------------------------------ proyecciones / cortes
def p_xy(m): return m.project().to_polygons()
def p_xz(m): return m.rotate((-90, 0, 0)).project().to_polygons()
def p_yz(m): return m.rotate((0, 0, -90)).rotate((-90, 0, 0)).project().to_polygons()
def c_xz(m, y): return m.translate((0, -y, 0)).rotate((-90, 0, 0)).slice(0.0).to_polygons()
def c_yz(m, x): return m.translate((-x, 0, 0)).rotate((0, 0, -90)).rotate((-90, 0, 0)).slice(0.0).to_polygons()


def capas_z(m, z0, z1, paso=0.5, saltar=()):
    """Cortes horizontales donde cambia la sección (de abajo hacia arriba)."""
    out, prev = [], None
    z = z0 + 0.05
    while z < z1:
        if any(a < z < b for a, b in saltar):
            z += paso; continue
        cs = m.slice(z)
        firma = cs.num_vert()          # solo cambios de forma (no traslaciones de rampas)
        if firma != prev:
            out.append((z, cs.to_polygons()))
            prev = firma
        z += paso
    return out


def vista_superior(v, partes, fc=GRIS2, saltar=()):
    """Vista desde arriba por capas: lo más alto tapa a lo más bajo. partes = [(manifold, estilo)]"""
    capas = []
    for m, est in partes:
        v.poligonos(p_xy(m), **est)    # silueta completa debajo
        b = m.bounding_box()
        for z, polys in capas_z(m, b[2], b[5], saltar=saltar):
            capas.append((z, polys, est))
        for a, bb in saltar:                       # tramo inclinado: su silueta, en su altura
            tramo = m ^ Manifold.cube((400, 400, bb - a)).translate((-200, -200, a))
            if s.vol(tramo) > 0:
                capas.append((a, tramo.project().to_polygons(), est))
    capas.sort(key=lambda c: c[0])
    for z, polys, est in capas:
        v.poligonos(polys, **est)


def vista_lateral_sel(v, m, ys, fc=GRIS2, lw=VIS):
    """Silueta + cortes XZ en las y indicadas (de la más lejana a la más cercana)."""
    v.poligonos(p_xz(m), fc=fc, lw=lw)
    for y in ys:
        cs = m.translate((0, -y, 0)).rotate((-90, 0, 0)).slice(0.0)
        if cs.num_vert():
            v.poligonos(cs.to_polygons(), fc=fc, lw=lw)


def vista_lateral(v, m, y_lejos, y_cerca, paso=0.5, fc=GRIS2, lw=VIS):
    """Vista XZ por capas en Y: se dibuja de lo más lejano a lo más cercano."""
    prev = None
    y = y_lejos
    sg = -1 if y_cerca < y_lejos else 1
    while (y - y_cerca) * sg < 0:
        cs = m.translate((0, -y, 0)).rotate((-90, 0, 0)).slice(0.0)
        firma = cs.num_vert()
        if firma != prev and cs.num_vert() > 0:
            v.poligonos(cs.to_polygons(), fc=fc, lw=lw)
            prev = firma
        y += sg * paso


def seccion(v, polys, lw=VIS):
    v.poligonos(polys, lw=lw, fc="white", hatch="////", z=3)


# ======================================================================================
#  HOJA 1 · SERVO SG90
# ======================================================================================
def hoja_servo(pdf):
    fig, fr = hoja("Servo SG90 y horn", 1, TOTAL, "2:1", "Medidas del plano del fabricante · tolerancia ±1,5 %")
    E = 2.0
    xe0, xe1 = s.SV_SHAFT_END - s.SV_L, s.SV_SHAFT_END          # -16,6 .. 5,9
    xc = (xe0 + xe1) / 2
    tl = s.SV_TAB_L; tz = s.SV_TAB_Z; tt = s.SV_TAB_T; bh = s.SV_BODY_H; top = s.SV_TOP; w = 11.8
    hx = [xc - s.SV_HOLE_SP / 2, xc + s.SV_HOLE_SP / 2]

    # ---------------- vista frontal (XZ)
    v = Vista(fig, 20, 150, 175, 125, -5, 15, E, "VISTA FRONTAL", fr)
    v.rect(xe0, 0, s.SV_L, bh, fc=GRIS2)
    v.rect(xc - tl / 2, tz, tl, tt, fc=GRIS2)
    v.rect(-8.8, bh, 8.8 + 5.9, top - bh, fc=GRIS2)
    v.rect(-2.3, top, 4.6, 3.2, fc="white")
    for x in hx:
        v.linea([x - 1, x - 1], [tz, tz + tt], lw=THIN, ls="--"); v.linea([x + 1, x + 1], [tz, tz + tt], lw=THIN, ls="--")
    v.eje(0, -2, 0, top + 3.2 + 2)
    v.cotah(xe0, xe1, -6, 0, 0)
    v.cotah(xc - tl / 2, xc + tl / 2, -12.5, tz, tz)
    v.cotah(0, xe1, top + 8, top + 3.2, bh)
    v.cotah(-8.8, 0, top + 8, bh, top + 3.2)
    v.cotah(xe0, -8.8, bh + 3.5, bh, bh)
    v.cotav(0, bh, xc - tl / 2 - 10, xe0, xe0)
    v.cotav(0, tz, xc - tl / 2 - 4, xc - tl / 2, xc - tl / 2)
    v.cotav(tz, tz + tt, xc - tl / 2 - 4, xc - tl / 2, xc - tl / 2, d=1)
    v.cotav(0, top, xe1 + 9, xe1, xe1)
    v.cotav(top, top + 3.2, xe1 + 9, 2.3, 2.3)
    v.cotav(0, top + 3.2, xe1 + 15.5, 2.3, 2.3)
    v.cotav(bh, top, -14, -8.8, -8.8)
    v.nota(-1.5, top + 3.2, -14, top + 9, "eje estriado Ø4,6")

    # ---------------- vista superior (XY)
    v = Vista(fig, 20, 52, 175, 92, -5, 0, E, "VISTA SUPERIOR", fr)
    v.rect(xc - tl / 2, -w / 2, tl, w, fc=GRIS2)
    v.rect(xe0, -w / 2, s.SV_L, w, fc="white")
    for x, sg in ((hx[0], -1), (hx[1], 1)):
        v.circulo(x, 0, 1.0, fc="white")
        v.rect(min(x, x + sg * 2.4), -0.65, 2.4, 1.3, fc="white", lw=0, ec="white", z=3)
        v.linea([x, x + sg * 2.4], [0.65, 0.65], lw=VIS); v.linea([x, x + sg * 2.4], [-0.65, -0.65], lw=VIS)
    v.circulo(0, 0, 5.9, fc=GRIS2)
    v.rect(-8.8, -2.5, 8.8, 5.0, fc=GRIS2, z=1)
    v.circulo(0, 0, 2.3, fc="white"); v.circulo(0, 0, 0.85, fc="white")
    v.cruz(0, 0, 9)
    v.cotah(hx[0], hx[1], -w / 2 - 6, 0, 0)
    v.cotav(-w / 2, w / 2, xc + tl / 2 + 5, xc + tl / 2, xc + tl / 2)
    v.cotav(-2.5, 2.5, -12, -8.8, -8.8)
    v.cotah(hx[1], xc + tl / 2, w / 2 + 5, 0, w / 2)
    v.diam(0, 0, 5.9, 120, 16)
    v.diam(0, 0, 2.3, 35, 30, "Ø4,6 (eje)")
    v.diam(0, 0, 0.85, -60, 24, "Ø1,7 (agujero del tornillo)")
    v.diam(hx[0], 0, 1.0, -120, 10, "Ø2,0")

    # ---------------- vista lateral (YZ)
    v = Vista(fig, 205, 150, 75, 125, 0, 15, E, "VISTA LATERAL", fr)
    v.rect(-w / 2, 0, w, bh, fc=GRIS2)
    v.rect(-w / 2, tz, w, tt, fc=GRIS2)
    v.rect(-w / 2, bh, w, top - bh, fc=GRIS2)
    v.rect(-2.3, top, 4.6, 3.2, fc="white")
    v.rect(-1.8, 4.5, 3.6, 1.2, fc="#555555")
    v.eje(0, -2, 0, top + 5)
    v.cotah(-w / 2, w / 2, -6, 0, 0)
    v.cotav(0, 4.5, w / 2 + 6, -1.8 + 3.6, w / 2)
    v.nota(1.8, 5.1, 10, 11, "salida del cable")

    # ---------------- horn
    v = Vista(fig, 190, 62, 75, 82, 6, 2, E, "HORN DE UN BRAZO", fr)
    tip = 16.0; rt = 1.9
    ang = np.linspace(-math.pi / 2, math.pi / 2, 30)
    pts = [(tip - rt + rt * math.cos(a), rt * math.sin(a)) for a in ang[::-1]]
    pts = [(0, 3.6)] + pts[::-1] + [(0, -3.6)]
    hull = [(3.6 * math.cos(a), 3.6 * math.sin(a)) for a in np.linspace(math.pi / 2, 3 * math.pi / 2, 30)]
    poly = [(0, 3.6)] + [(tip - rt + rt * math.cos(a), rt * math.sin(a)) for a in np.linspace(math.pi / 2, -math.pi / 2, 30)] + [(0, -3.6)] + hull[::-1][1:]
    v.poligonos([poly], fc="white")
    v.circulo(0, 0, 3.6, lw=THIN, ls="--")
    v.circulo(0, 0, 0.9, fc="white")
    for i in range(6):
        v.circulo(5.5 + i * 1.75, 0, 0.45, lw=THIN)
    v.cotah(-3.6, tip, -7, 0, 0, txt="19,6")
    v.diam(0, 0, 3.6, 115, 12, "Ø7,2 (cubo)")
    # vista de costado del horn
    yo = 12
    v.rect(-3.6, yo, 7.2, 4.0, fc=GRIS2)
    v.rect(3.6, yo + 2.4, tip - 3.6, 1.6, fc=GRIS2)
    v.cotav(yo, yo + 4.0, -7, -3.6, -3.6)
    v.cotav(yo + 2.4, yo + 4.0, tip + 4, tip, tip)
    v.nota(0, yo, 6, yo - 3.5, "el cubo va hacia el servo")

    # ---------------- datos
    fr.text(292, 270, "DATOS DEL SG90", fontsize=9, weight="bold")
    datos = [("Peso", "9 g"), ("Tensión", "4,8 – 6,0 V"), ("Par de bloqueo", "1,4 kg·cm (4,8 V) · 1,9 kg·cm (6 V)"),
             ("Velocidad", "0,09 s / 60° (4,8 V)"), ("Señal", "PWM 50 Hz, 0,5 – 2,5 ms"),
             ("Recorrido", "180°"), ("Cables", "marrón GND · rojo +V · naranja señal (D9)")]
    for i, (a, b) in enumerate(datos):
        fr.text(292, 262 - i * 6.2, a, fontsize=7.5, color="#444")
        fr.text(322, 262 - i * 6.2, b, fontsize=7.5)
    fr.text(292, 210, "NOTAS", fontsize=9, weight="bold")
    notas = ["El eje está a 5,9 mm del extremo del cuerpo.",
             "Las orejas tienen ranura abierta: los tornillos M2",
             "  van en guías de Ø1,7 en el soporte.",
             "Algunas marcas miden 12,15 de ancho; el soporte",
             "  tiene 12,5 para que entren ambas.",
             "Horn: cubo hacia el servo, paleta arriba.",
             "Tornillo del horn: autorroscante 2 × 6 mm."]
    for i, t in enumerate(notas):
        fr.text(292, 202 - i * 6.2, t, fontsize=7.5)
    pdf.savefig(fig); fig.savefig(OUT + "hoja1_servo.png", dpi=110); plt.close(fig)


# ======================================================================================
#  HOJA 2 · SOPORTE
# ======================================================================================
def hoja_soporte(pdf):
    fig, fr = hoja("Soporte (base con torre)", 2, TOTAL, "1:1")
    E = 1.0
    xe0, xe1, xc, wx0, wx1, wy = s.base_dims()
    b = BASE.bounding_box()
    zr0 = s.ZR0; zp = zr0 - 26.0
    hx = [xc - s.SV_HOLE_SP / 2, xc + s.SV_HOLE_SP / 2]
    my = wy + 2.75

    # ---------------- vista frontal (XZ)
    v = Vista(fig, 20, 60, 150, 215, 2, 52, E, "VISTA FRONTAL", fr)
    v.poligonos(p_xz(BASE), fc=GRIS2)
    v.poligonos(p_xz(SERVO), lw=THIN, ec="#777777", fc="none", z=1)
    v.eje(0, -3, 0, b[5] + 4)
    v.cotav(0, b[5], b[0] - 10, b[0], s.RING_OD / 2 * -1, d=2)
    v.cotav(zr0, zr0 + s.RING_H, 18, s.RING_OD / 2, s.RING_OD / 2)
    v.cotav(0, zr0, 28, 0, s.RING_OD / 2, d=2)
    v.cotav(0, s.PLATE_T, b[3] + 6, b[3], b[3])
    v.cotav(0, s.PLATE_T + s.SV_TAB_Z, wx1 + 13, wx1, wx1)
    v.cotav(0, s.PLATE_T + s.SV_TAB_Z + s.SV_TAB_T, wx1 + 21, wx1, wx1)
    v.cotah(wx0 - 7.5, wx0 - 0.5, zp + 7, zp, zp)
    v.cotah(b[0], b[3], -8, 0, 0)
    v.cotah(b[0], 0, -15, 0, 0)
    v.cotah(-s.RING_OD / 2, s.RING_OD / 2, b[5] + 6, b[5], b[5])
    v.angulo(-s.RING_OD / 2 - 3, zr0 - 3, 10, 180, 225, "45°")
    v.nota(wx0 - 4.4, 41.6, -48, 46, "ranuras para precintos\n1,6 × 3,2 (z = 40 y 62)")
    v.nota(0, zr0 + 3, 16, zr0 + 16, "anillo guía de cables\n(centrado sobre el eje)")
    v.nota(-3, 24, -45, 24, "servo (referencia)")

    # ---------------- vista superior (XY)
    v = Vista(fig, 180, 150, 110, 125, 2, 0, E, "VISTA SUPERIOR", fr)
    vista_superior(v, [(BASE, dict(fc=GRIS2))], saltar=[(zp - 6.5, zr0 - 0.3)])
    v.cruz(0, 0, 40)
    v.cotah(b[0], b[3], b[1] - 8, b[1], b[1])
    v.cotav(b[1], b[4], b[3] + 7, b[3], b[3])
    v.cotah(hx[0], hx[1], wy + 5.5 + 5, my, my)
    v.cotav(-my, my, wx0 - 13, hx[0], hx[0])
    v.cotah(xe0 - s.FIT_X, xe1 + s.FIT_X, -16.5, -wy + 2.1, -wy + 2.1, txt="22,9")
    v.cotav(-6.25, 6.25, xe0 - 6, xe0, xe0, txt="12,5")
    a = math.radians(-60)
    v.nota(34 * math.cos(a), 34 * math.sin(a), 38, -34, "R34 (pie antivuelco)")
    v.angulo(0, 0, 22, -52, 52, "104°")
    v.nota(hx[0], -my, -30, -34, "4× Ø3,4 (M3)")
    v.nota(28.5 * math.cos(math.radians(43)), 28.5 * math.sin(math.radians(43)), 30, 36, "3× Ø3,4 a R28,5")
    v.nota(0, 0, 14, -40, "eje del servo")
    v.nota(-s.RING_OD / 2 + 0.5, 4, -30, 36, "anillo guía (arriba)")
    corte_letras(v, wx0 + 0.4, -wy - 5, wx0 + 0.4, -wy - 12, "E")

    # ---------------- vista lateral (YZ)
    v = Vista(fig, 300, 60, 105, 215, 0, 52, E, "VISTA LATERAL", fr)
    v.poligonos(p_yz(BASE), fc=GRIS2)
    v.eje(0, -3, 0, b[5] + 4)
    v.cotah(-13.85, 13.85, -8, 0, 0, txt="27,7")
    v.cotah(wy, wy + s.POST_W, zp + 7, zp, zp)
    v.cotah(-wy, wy, s.PLATE_T + s.SV_TAB_Z + s.SV_TAB_T + 6, s.PLATE_T + s.SV_TAB_Z + s.SV_TAB_T,
            s.PLATE_T + s.SV_TAB_Z + s.SV_TAB_T, txt=num(2 * wy))

    # ---------------- detalle del anillo (2:1)
    v = Vista(fig, 180, 60, 60, 70, 0, 0, 2.0, "DETALLE ANILLO (2:1)", fr)
    v.circulo(0, 0, s.RING_OD / 2, fc=GRIS2); v.circulo(0, 0, s.RING_ID / 2, fc="white")
    v.rect(-s.RING_GAP / 2, -s.RING_OD / 2 - 0.2, s.RING_GAP, s.RING_OD / 2 - s.RING_ID / 2 + 0.4, fc="white", lw=0, ec="white", z=3)
    v.linea([-s.RING_GAP / 2] * 2, [-s.RING_OD / 2, -s.RING_ID / 2]); v.linea([s.RING_GAP / 2] * 2, [-s.RING_OD / 2, -s.RING_ID / 2])
    v.cruz(0, 0, 18)
    v.diam(0, 0, s.RING_ID / 2, 135, 14, "Ø6,5")
    v.diam(0, 0, s.RING_OD / 2, 45, 10, "Ø13")
    v.nota(0.4, -s.RING_OD / 2, 6, -11, "ranura 1,4\n(cables a presión)")

    # ---------------- corte E-E: ranura del cable del servo (1,5:1)
    xe = wx0 + 0.4
    v = Vista(fig, 246, 90, 54, 50, 0, 11, 1.5, "CORTE E-E (1,5:1)", fr)
    seccion(v, c_yz(BASE, xe))
    v.cotah(s.CSLOT_Y0, s.CSLOT_Y1, s.PLATE_T + s.SV_TAB_Z + s.SV_TAB_T + 3, s.PLATE_T, s.PLATE_T)
    v.cotav(0, s.PLATE_T, 15.5, 13.85, 13.85)
    v.nota((s.CSLOT_Y0 + s.CSLOT_Y1) / 2, 8, -16, 26, "ranura abierta\npara el cable", fs=6.5, ha="center")
    fr.text(250, 78, "NOTAS", fontsize=9, weight="bold")
    for i, t in enumerate(["El servo entra desde arriba; orejas con 2 tornillos M2 (Ø1,7 guía).",
                           "Cable del servo: sale hacia atrás por una ranura abierta por arriba (corte E-E),",
                           "  del lado opuesto a la torre: el cable ya soldado se acuesta desde arriba.",
                           "Imprimir con la placa sobre la cama, sin soportes.",
                           "Medidas del soporte para brazo L = 120 (pie R34)."]):
        fr.text(250, 72 - i * 4.6, t, fontsize=7.2)
    pdf.savefig(fig); fig.savefig(OUT + "hoja2_soporte.png", dpi=110); plt.close(fig)


# ======================================================================================
#  HOJA 3 · BRAZO Y CABEZAL
# ======================================================================================
def ldr_led_xz(v, y):
    """Dibuja LDR y LED (cortados) en el plano XZ de la cabina en y."""
    z0 = s.FLOOR_GAP + s.LDR_LEDGE
    v.rect(L - 2.55, z0 + 0.35, 5.1, 2.05, fc="#f3e7c9", lw=THIN, z=5)
    v.rect(L - 2.55, z0, 5.1, 0.35, fc="#c2541c", lw=THIN, z=5)
    for dx in (-0.9, 0.9):
        v.linea([L + dx, L + dx], [z0 + 2.4, s.ZT - 3.0], lw=0.6, c="#777777", z=5)
    t = math.radians(s.LED_TILT); u = np.array([math.sin(t), math.cos(t)]); n = np.array([math.cos(t), -math.sin(t)])
    o = np.array([L + s.LED_DX, s.FLOOR_GAP])
    P = lambda a, b: o + a * u + b * n
    body = [P(2.9, -2.5), P(8.0, -2.5), P(8.0, 2.5), P(2.9, 2.5)]
    dome = [P(2.9 - 2.5 * math.cos(a), 2.5 * math.sin(a)) for a in np.linspace(-math.pi / 2, math.pi / 2, 24)]
    v.poligonos([body + dome[::-1]], fc="#ffd9d9", lw=THIN, z=5)
    v.poligonos([[P(8, -2.9), P(9, -2.9), P(9, 2.9), P(8, 2.9)]], fc="#ffb3b3", lw=THIN, z=5)
    for b in (-1.0, 1.0):
        a0, a1 = P(9, b), P(14.5, b)
        v.linea([a0[0], a1[0]], [a0[1], a1[1]], lw=0.6, c="#777777", z=5)
    return P


def hoja_brazo(pdf):
    fig, fr = hoja("Brazo L = 120 y cabezal", 3, TOTAL, "1:1 (detalles indicados)")
    yc = s.LDR_SP / 2
    zt, za, zb = s.ZT, s.ZA, s.FLOOR_GAP
    bb = ARM.bounding_box()

    # ---------------- vista superior 1:1
    v = Vista(fig, 20, 212, 222, 66, 64, 0, 1.0, None)
    fr.text(131, 209, "VISTA SUPERIOR", ha="center", va="top", fontsize=9, weight="bold")
    vista_superior(v, [(ARM, dict(fc=GRIS2))])
    v.eje(-6.5, 0, bb[3], 0); v.cruz(0, 0, 9); v.cruz(L, yc, 5); v.cruz(L, -yc, 5)
    v.cotah(0, L, 27, 0, yc + 4.1)
    v.cotah(bb[0], bb[3], -24, 0, bb[1])
    v.cotav(-yc, yc, L - 18, L - 4, L - 4)
    v.cotav(bb[1], bb[4], bb[3] + 7, bb[3] - 4, bb[3] - 4)
    v.cotav(-s.ARM_W / 2, s.ARM_W / 2, 60, 60, 60)
    v.diam(0, 0, s.HUB_R, 135, 9, "Ø13")
    v.nota(75, s.GROOVE_W / 2 - 1, 82, 15, "canal de cables 6,8 × 2,4")
    v.cotah(0, s.GROOVE_X0, -10, 0, -s.GROOVE_W / 2)
    corte_letras(v, 21.8, -9, 21.8, 9, "A")
    corte_letras(v, L - 9, yc + 0.3, L + 22, yc + 0.3, "B")
    corte_letras(v, L - 0.5, -19, L - 0.5, 19, "C")

    # ---------------- vista lateral 1:1
    v = Vista(fig, 20, 140, 222, 64, 64, 22, 1.0, None)
    fr.text(131, 137, "VISTA LATERAL", ha="center", va="top", fontsize=9, weight="bold")
    vista_lateral_sel(v, ARM, [yc + 0.3, 0.0, -yc - 0.3])
    v.eje(0, za - 3, 0, zt + 3)
    v.linea([-10, 150], [0, 0], lw=THIN, c="#888888"); v.texto(150, 1.5, "piso", fs=6.5, ha="right", c="#666")
    v.cotav(za, zt, -12, -6.5, -6.5)
    v.cotav(zb, zt, bb[3] + 7, bb[3] - 6, bb[3] - 6, d=2)
    v.cotav(0, zb, L - 12, L - 4.1, L - 4.1)
    v.angulo(L + s.LED_DX, zb, 9, 90 - s.LED_TILT, 90, "35°")
    v.nota(L + 12, 22, L + 22, 30, "tubo del LED")
    v.nota(L - 4.1, 15, L - 22, 13, "cabina del LDR")

    # ---------------- corte A-A 5:1
    v = Vista(fig, 22, 62, 72, 66, 0, za + 2.5, 5.0, "CORTE A-A (5:1)", fr)
    seccion(v, c_yz(ARM, 21.8))
    for (yy, zz) in s.posiciones_cables():
        v.circulo(yy, zz, 0.5, lw=THIN, fc="#f0c060", z=5)
    v.cotah(-s.ARM_W / 2, s.ARM_W / 2, za - 2, za, za)
    v.cotav(za, zt, s.ARM_W / 2 + 2, s.ARM_W / 2, s.ARM_W / 2)
    v.cotah(-s.GROOVE_W / 2, s.GROOVE_W / 2, zt + 2.2, zt - s.GROOVE_D, zt - s.GROOVE_D)
    v.cotav(zt - s.GROOVE_D, zt, -s.ARM_W / 2 - 2, -s.GROOVE_W / 2, -s.GROOVE_W / 2)
    v.cotah(-s.GROOVE_W / 2 + s.LIP, s.GROOVE_W / 2 - s.LIP, zt + 4.0, zt, zt)
    v.nota(s.GROOVE_W / 2 - 0.55, zt - 0.5, 6.8, zt + 2.6, "pestaña 1,1")
    v.nota(0, zt - 1.6, -5.2, za - 1.8, "10 cables Ø1 (6 + 4)")

    # ---------------- corte D-D por el eje del horn 3:1
    v = Vista(fig, 102, 62, 138, 66, 12, za + 1, 3.0, "CORTE D-D POR EL EJE DEL HORN (3:1)", fr)
    secc = ARM ^ Manifold.cube((40, 40, 60)).translate((-8, -20, 0))
    seccion(v, c_xz(secc, 0.0))
    v.poligonos(c_xz(HORN, 0.0), fc="#f6f6f6", lw=THIN, z=2)
    v.poligonos(c_xz(SERVO ^ Manifold.cube((40, 40, 60)).translate((-8, -20, 0)), 0.0), fc="none", lw=THIN, ec="#888", z=1)
    v.rect(-1.0, za + s.POCKET_D + s.ROOF_T - 6.0 + 2.3 + 1.5, 2.0, 6.0, fc="#bbbbbb", lw=THIN, z=6)
    v.rect(-1.9, za + s.POCKET_D + s.ROOF_T, 3.8, 1.4, fc="#bbbbbb", lw=THIN, z=6)
    v.eje(0, za - 6, 0, zt + 4)
    v.cotav(za, za + s.POCKET_D, -9.5, -s.HUB_R, -s.HUB_R, d=2)
    v.cotav(za + s.POCKET_D, za + s.POCKET_D + s.ROOF_T, 5.5, 2.2, 2.2)
    v.cotah(-s.SCREW_HEAD_D / 2, s.SCREW_HEAD_D / 2, zt + 2.5, zt, zt)
    v.cotah(0, s.HORN_PAD_L, za - 4.5, za, za, txt="16,4")
    v.nota(10, za + 0.6, 22, za - 2.5, "hueco del horn (paleta)")
    v.nota(-2, za - 1.5, -6, za - 5, "horn", ha="right")
    v.nota(1.9, zt - 0.3, 8, zt + 3.5, "avellanado Ø4,4")
    v.nota(-1.0, za + 2.0, -7.5, za + 7, "tornillo 2 × 6", ha="right")

    # ---------------- corte B-B 2:1
    v = Vista(fig, 248, 140, 96, 140, L - 1, 17, 2.0, "CORTE B-B (2:1)", fr)
    seccion(v, c_xz(ARM, yc + 0.3))
    P = ldr_led_xz(v, yc)
    v.linea([L - 25, L + 22], [0, 0], lw=VIS); v.texto(L + 21, -1.6, "piso", fs=6.5, ha="right")
    z0 = zb + s.LDR_LEDGE
    v.cotav(0, z0, L - 16, L - 2.55, L - 2.55, txt="8")
    v.cotav(0, zb, L - 21, L - 4.1, L - 4.1, txt="7")
    v.cotav(z0, zt, L + 6.5, L + 2.55, L + 4.1, d=2)
    v.cotav(zb, z0, L - 6.5, L - 1.8, L - 1.8, txt="1")
    v.cotah(L - 2.75, L + 2.75, zt + 3, zt - 3.2, zt - 3.2, txt="Ø5,5")
    v.nota(L + 0.5, zb + 0.3, L + 8, -4.5, "ventana Ø3,6")
    v.nota(L + 1.5, z0 + 0.3, L + 12, 4.5, "LDR (cara sensible abajo)")
    pp = P(8.5, 2.9)
    v.nota(pp[0], pp[1], L + 9.5, 24, "LED Ø5:\nreborde en el\nescalón Ø6,3", ha="left")
    v.nota(L - 3.5, zt - 1.5, L - 18, zt + 4, "salida de patitas", ha="right")

    # ---------------- corte C-C 2:1
    v = Vista(fig, 348, 140, 60, 140, 0, 17, 2.0, "CORTE C-C (2:1)", fr)
    seccion(v, c_yz(ARM, L - 0.5))
    v.linea([-15, 15], [0, 0], lw=VIS)
    v.cotah(-yc, yc, zt + 4, zt, zt, txt="20")
    v.cotah(-yc - s.COL_R, -yc + s.COL_R, -4, zb, zb, txt="Ø8,2")
    v.nota(yc + 2.0, 20, 4, 25, "cabina Ø5,5", ha="right")
    v.eje(-yc, zb - 3, -yc, zt + 2); v.eje(yc, zb - 3, yc, zt + 2)

    # ---------------- vista inferior del cubo 2:1
    v = Vista(fig, 248, 64, 84, 70, 7, 0, 2.0, "CARA INFERIOR DEL CUBO (2:1)", fr)
    det = ARM ^ Manifold.cube((30, 30, 60)).translate((-8, -15, 0))
    v.poligonos(det.translate((0, 0, -za)).slice(0.05).to_polygons(), fc=GRIS2)
    v.cruz(0, 0, 10)
    v.diam(0, 0, s.HORN_HUB_D / 2 + s.HORN_CLR, 125, 10, "Ø7,4")
    v.cotah(0, s.HORN_PAD_L, -8.5, 0, -2.1, txt="16,4")
    v.cotav(-(s.HORN_PAD_W1 / 2 + s.HORN_CLR), s.HORN_PAD_W1 / 2 + s.HORN_CLR, s.HORN_PAD_L + 3, s.HORN_PAD_L - 1, s.HORN_PAD_L - 1, txt="4")
    v.nota(4, 0.5, 8, 9.5, "profundidad 1,25")
    v.circulo(0, 0, s.SCREW_D / 2, lw=THIN, ls="--")
    v.diam(0, 0, s.SCREW_D / 2, -60, 12, "Ø2,4 (pasante arriba)")

    fr.text(338, 120, "NOTAS", fontsize=9, weight="bold")
    for i, tt in enumerate(["Holgura del horn: 0,10", "por lado.", "LDR a 20 mm = ancho", "de la cinta (19 mm).",
                            "Cabina: Ø5,5, tope 1,", "ventana Ø3,6.", "Imprimir boca abajo,", "sin soportes."]):
        fr.text(338, 113 - i * 5.6, tt, fontsize=7.2)
    pdf.savefig(fig); fig.savefig(OUT + "hoja3_brazo.png", dpi=110); plt.close(fig)


# ======================================================================================
#  HOJA 4 · CONJUNTO
# ======================================================================================
def hoja_conjunto(pdf):
    fig, fr = hoja("Conjunto montado", 4, TOTAL, "1:1")
    yc = s.LDR_SP / 2
    bb = ARM.bounding_box(); bbase = BASE.bounding_box()
    zt, zr0 = s.ZT, s.ZR0

    # ---------------- vista lateral
    v = Vista(fig, 18, 58, 212, 226, 52, 55, 1.0, None)
    fr.text(124, 55, "VISTA LATERAL (brazo a 90°)", ha="center", va="top", fontsize=9, weight="bold")
    v.linea([-45, 152], [0, 0], lw=VIS); v.texto(150, -2, "piso", fs=6.5, ha="right")
    v.poligonos(p_xz(BASE), fc=GRIS2)
    v.poligonos(p_xz(SERVO), fc="#dfe8f5", lw=THIN, z=4)
    v.poligonos(p_xz(HORN), fc="white", lw=THIN, z=5)
    vista_lateral_sel(v, ARM, [yc + 0.3, 0.0, -yc - 0.3])
    v.eje(0, -4, 0, bbase[5] + 4)
    v.cotav(0, bbase[5], -42, bbase[0], -s.RING_OD / 2, d=2)
    v.cotav(0, zr0, 14, s.RING_OD / 2, s.RING_OD / 2, d=2)
    v.cotav(0, zt, 50, 20, 20, d=2)
    v.cotav(0, s.FLOOR_GAP, L - 14, L - 4.1, L - 4.1, txt="7")
    v.cotah(0, L, zt + 9, zt, zt)
    v.cotah(bbase[0], bb[3], -16, 0, 0)
    v.cotah(bbase[0], 0, -9, 0, 0)
    v.cotah(0, bb[3], -9, 0, 0)
    v.nota(-3, 15, -40, 30, "servo SG90", ha="right")
    v.nota(L + 3, 4, L + 18, 16, "LDR + LED\n(cabezal)")
    v.nota(4, zr0 + 4, 26, zr0 + 12, "anillo guía de cables")

    # ---------------- vista superior con el barrido
    v = Vista(fig, 236, 58, 170, 220, 55, 0, 0.8, None)
    fr.text(321, 283, "VISTA SUPERIOR · BARRIDO 60°–120° (escala 1:1,25)", ha="center", va="top", fontsize=9, weight="bold")
    for ang in (-30, 30):
        v.poligonos(p_xy(ARM.rotate((0, 0, ang))), fc="none", lw=THIN, ec="#999999", z=1)
    vista_superior(v, [(BASE, dict(fc=GRIS2)), (SERVO, dict(fc="#dfe8f5", lw=THIN)), (ARM, dict(fc=GRIS2))],
                   saltar=[(s.ZR0 - 32.5, s.ZR0 - 0.3)])
    v.eje(0, 0, 145, 0)
    for ang in (-30, 30):
        a = math.radians(ang); v.eje(0, 0, 145 * math.cos(a), 145 * math.sin(a))
    from matplotlib.patches import Arc
    v.ax.add_patch(Arc((0, 0), 2 * L, 2 * L, theta1=-30, theta2=30, lw=0.5, ls="--", color=INK, zorder=4))
    v.angulo(0, 0, 70, 0, 30, "30° (120°)")
    v.angulo(0, 0, 70, -30, 0, "30° (60°)")
    v.cotav(-L * 0.5, L * 0.5, L + 30, L * math.cos(math.radians(30)), L * math.cos(math.radians(30)),
            txt="cobertura 120 (±60)")
    v.nota(L * math.cos(math.radians(22)), L * math.sin(math.radians(22)), 150, 96, "trayectoria de los LDR (R120)", ha="right")
    v.cotav(bbase[1], bbase[4], bbase[0] - 7, bbase[0] + 5, bbase[0] + 5)
    v.nota(0, 0, -30, -50, "eje del servo")

    fr.text(250, 110, "MEDIDAS GENERALES", fontsize=9, weight="bold")
    filas = [("Largo total", num(bb[3] - bbase[0]) + " mm"), ("Alto total (torre)", num(bbase[5], 2) + " mm"),
             ("Ancho del soporte", num(bbase[4] - bbase[1]) + " mm"), ("Eje → centro de los LDR", "120 mm"),
             ("Cara superior del brazo", num(zt, 2) + " mm"), ("Ventanas de los LDR al piso", "7 mm"),
             ("Recorrido", "60° – 120° (±30°)"), ("Peso brazo + cables", "≈ 14 g")]
    for i, (a, b2) in enumerate(filas):
        fr.text(250, 102 - i * 5.8, a, fontsize=7.4, color="#444"); fr.text(330, 102 - i * 5.8, b2, fontsize=7.4)
    pdf.savefig(fig); fig.savefig(OUT + "hoja4_conjunto.png", dpi=110); plt.close(fig)


# ======================================================================================
#  HOJA 5 · ESQUEMA ELÉCTRICO
# ======================================================================================
def hoja_esquema(pdf):
    fig, fr = hoja("Esquema eléctrico (cableado actual)", 5, TOTAL, "s/e", "Arduino Nano · 2 LDR · 2 LED blancos · servo SG90 con fuente aparte")
    A = fr
    C_W = INK
    def w(*pts, c=C_W, lw=0.9, ls="-"):
        xs, ys = zip(*pts); A.plot(xs, ys, color=c, lw=lw, ls=ls, solid_capstyle="round", zorder=3)
    def dot(x, y): A.add_patch(Circle((x, y), 0.9, color=INK, zorder=5))
    def txt(x, y, t, fs=7.5, ha="left", va="center", wgt="normal", c=INK):
        A.text(x, y, t, fontsize=fs, ha=ha, va=va, weight=wgt, color=c, zorder=6)
    def res_v(x, ytop, ybot, label, val):
        m = (ytop + ybot) / 2
        w((x, ytop), (x, m + 5)); w((x, m - 5), (x, ybot))
        A.add_patch(Rectangle((x - 1.6, m - 5), 3.2, 10, fill=True, fc="white", ec=INK, lw=0.9, zorder=4))
        txt(x + 3, m + 1.8, label, fs=7.5, wgt="bold"); txt(x + 3, m - 2.2, val, fs=7)
    def res_h(x0, x1, y, label, val):
        m = (x0 + x1) / 2
        w((x0, y), (m - 5, y)); w((m + 5, y), (x1, y))
        A.add_patch(Rectangle((m - 5, y - 1.6), 10, 3.2, fill=True, fc="white", ec=INK, lw=0.9, zorder=4))
        txt(m, y + 3.4, label, fs=7.2, ha="center", wgt="bold"); txt(m, y - 3.4, val, fs=6.8, ha="center")
    def flecha(x0, y0, x1, y1):
        A.annotate("", xy=(x1, y1), xytext=(x0, y0), arrowprops=dict(arrowstyle="-|>,head_length=0.35,head_width=0.15", lw=0.7, color=INK), zorder=6)
    def ldr(x, ytop, ybot, label):
        m = (ytop + ybot) / 2
        w((x, ytop), (x, m + 5)); w((x, m - 5), (x, ybot))
        A.add_patch(Rectangle((x - 1.6, m - 5), 3.2, 10, fill=True, fc="white", ec=INK, lw=0.9, zorder=4))
        A.add_patch(Circle((x, m), 6.2, fill=False, ec=INK, lw=0.7, zorder=4))
        flecha(x - 12, m + 7, x - 5, m + 2.5); flecha(x - 12, m + 3, x - 5, m - 1.5)
        txt(x + 8, m + 2, label, wgt="bold"); txt(x + 8, m - 2.2, "GL5528 (10k)", fs=6.8)
    def led(x, ytop, ybot, label):
        m = (ytop + ybot) / 2
        w((x, ytop), (x, m + 2.6)); w((x, m - 2.6), (x, ybot))
        A.add_patch(Polygon([(x - 3, m + 2.6), (x + 3, m + 2.6), (x, m - 2.6)], closed=True, fc="white", ec=INK, lw=0.9, zorder=4))
        w((x - 3, m - 2.6), (x + 3, m - 2.6))
        flecha(x + 3.5, m + 1, x + 8, m + 4); flecha(x + 3.5, m - 1.5, x + 8, m + 1.5)
        txt(x - 4.5, m, label, ha="right", wgt="bold", fs=7.2)
    def gnd(x, y):
        w((x, y), (x, y - 3))
        for i, h in enumerate((3.2, 2.0, 0.8)):
            w((x - h, y - 3 - i * 1.3), (x + h, y - 3 - i * 1.3))
    def v5(x, y):
        w((x, y), (x, y + 3)); w((x - 2.5, y + 3), (x + 2.5, y + 3))
        txt(x, y + 6, "+5V", fs=7, ha="center", wgt="bold")
    def num_cable(x, y, n):
        A.add_patch(Circle((x, y), 2.4, fc="white", ec=INK, lw=0.6, zorder=6))
        txt(x, y, str(n), fs=6.5, ha="center", wgt="bold")

    # ---------------- zonas
    A.add_patch(Rectangle((20, 112), 98, 165, fill=False, ls=(0, (5, 3)), lw=0.8, ec="#555", zorder=1))
    txt(23, 273, "CABEZAL (en el brazo)", fs=8.5, wgt="bold")
    A.add_patch(Rectangle((128, 112), 160, 165, fill=False, ls=(0, (5, 3)), lw=0.8, ec="#555", zorder=1))
    txt(131, 273, "PLACA", fs=8.5, wgt="bold")
    txt(123, 116, "mazo de 10 cables", fs=6.8, ha="center", c="#444")
    A.text(123, 194, "10 cables por el brazo y la torre", fontsize=6.8, rotation=90, ha="center", va="center", color="#444")

    # ---------------- Arduino Nano
    ax0, ay0, aw, ah = 300, 150, 72, 115
    A.add_patch(Rectangle((ax0, ay0), aw, ah, fill=True, fc="#eef3fa", ec=INK, lw=1.1, zorder=2))
    txt(ax0 + aw / 2, ay0 + ah - 9, "ARDUINO NANO", fs=9, ha="center", wgt="bold")
    txt(ax0 + aw / 2, ay0 + ah - 15, "(clon CH340 · USB a la PC)", fs=6.8, ha="center")
    pins = {"A0": 232, "A1": 174, "D9": 160}
    for n, y in pins.items():
        w((ax0 - 4, y), (ax0, y)); txt(ax0 + 2, y, n, fs=7.5, wgt="bold")
    w((ax0 + 20, ay0 + ah), (ax0 + 20, ay0 + ah + 4)); txt(ax0 + 20, ay0 + ah - 3, "5V", fs=7.5, ha="center", wgt="bold")
    v5(ax0 + 20, ay0 + ah + 4)
    w((ax0 + 40, ay0), (ax0 + 40, ay0 - 4)); txt(ax0 + 40, ay0 + 3, "GND", fs=7.5, ha="center", wgt="bold")
    txt(ax0 + aw / 2, ay0 + 40, "D5 / D6 libres\n(PWM para los LED\nsi querés regular brillo)", fs=6.4, ha="center", c="#555")

    # ---------------- LDR izquierdo
    x = 48
    ldr(x, 258, 236, "LDR izq.")
    w((x, 258), (x, 264), (138, 264)); v5(138, 264); num_cable(110, 264, 1)
    dot(x, 236)
    w((x, 236), (x, 232), (ax0 - 4, 232)); num_cable(110, 232, 3)
    w((x, 232), (x, 224), (150, 224)); num_cable(102, 224, 2)
    res_v(150, 224, 200, "R1", "4,7 kΩ"); gnd(150, 200)
    # ---------------- LDR derecho
    ldr(x, 200, 178, "LDR der.")
    w((x, 200), (x, 206), (138, 206)); v5(138, 206); num_cable(110, 206, 4)
    dot(x, 178)
    w((x, 178), (x, 174), (ax0 - 4, 174)); num_cable(110, 174, 6)
    w((x, 174), (x, 166), (255, 166)); num_cable(102, 166, 5)
    res_v(255, 166, 142, "R2", "4,7 kΩ"); gnd(255, 142)
    # ---------------- LED
    led(45, 150, 132, "LED izq.\n(blanco)")
    led(85, 150, 132, "LED der.\n(blanco)")
    w((45, 150), (45, 161), (172, 161)); num_cable(110, 161, 7)
    res_h(172, 200, 161, "R_LED1", "150–220 Ω"); w((200, 161), (206, 161)); v5(206, 161)
    w((85, 150), (85, 154), (212, 154)); num_cable(102, 154, 9)
    res_h(212, 240, 154, "R_LED2", "150–220 Ω"); w((240, 154), (244, 154)); v5(244, 154)
    w((45, 132), (45, 122), (140, 122)); num_cable(110, 122, 8); gnd(140, 122)
    w((85, 132), (85, 128), (160, 128)); num_cable(102, 128, 10); gnd(160, 128)

    # ---------------- alimentación del servo: entrada -> capacitores -> salida al servo
    A.add_patch(Rectangle((22, 62), 30, 42, fill=True, fc="#f4f4f4", ec=INK, lw=1.0, zorder=2))
    txt(37, 94, "ENTRADA", fs=7.5, ha="center", wgt="bold"); txt(37, 89, "alimentación", fs=6.6, ha="center")
    txt(37, 78, "VCC 5–6 V\n≥ 1 A", fs=7, ha="center")
    txt(54, 101, "+", fs=8, wgt="bold"); txt(54, 65, "−", fs=9, wgt="bold")
    w((52, 98), (162, 98)); w((52, 68), (162, 68))
    dot(95, 98); dot(95, 68); dot(130, 98); dot(130, 68)
    # electrolítico
    w((95, 98), (95, 85)); w((95, 81), (95, 68))
    w((90.5, 85), (99.5, 85), lw=1.3)
    A.add_patch(Arc((95, 77.2), 11, 7, theta1=40, theta2=140, lw=1.3, color=INK, zorder=4))
    txt(91, 88, "+", fs=7, ha="center", wgt="bold")
    txt(101, 84, "C1 electrolítico", fs=7, wgt="bold"); txt(101, 80, "470–1000 µF", fs=6.8)
    # cerámico
    w((130, 98), (130, 84.5)); w((130, 81.5), (130, 68))
    w((125.5, 84.5), (134.5, 84.5), lw=1.3); w((125.5, 81.5), (134.5, 81.5), lw=1.3)
    txt(136, 84, "C2 cerámico", fs=7, wgt="bold"); txt(136, 80, "100 nF", fs=6.8)
    txt(112.5, 106, "C1 y C2 en paralelo, entre la entrada y la salida", fs=6.4, ha="center", c="#444")
    # salida al servo
    A.add_patch(Rectangle((162, 62), 22, 42, fill=True, fc="#f4f4f4", ec=INK, lw=1.0, zorder=2))
    txt(173, 94, "SALIDA", fs=7.5, ha="center", wgt="bold"); txt(173, 89, "al servo", fs=6.6, ha="center")
    txt(165, 98, "+", fs=8, wgt="bold"); txt(165, 68, "−", fs=9, wgt="bold")
    w((184, 98), (210, 98)); w((184, 68), (210, 68))
    # servo
    A.add_patch(Rectangle((210, 62), 40, 42, fill=True, fc="#e6eefb", ec=INK, lw=1.0, zorder=2))
    A.add_patch(Circle((238, 83), 7, fill=False, ec=INK, lw=0.9, zorder=3)); txt(238, 83, "M", fs=10, ha="center", wgt="bold")
    txt(213, 98, "rojo +", fs=6.6); txt(213, 83, "naranja\nseñal", fs=6.6); txt(213, 69, "marrón −", fs=6.6)
    txt(230, 107.5, "SERVO SG90", fs=7.5, ha="center", wgt="bold")
    w((250, 83), (285, 83), (285, 160), (ax0 - 4, 160))
    # negativo de los capacitores -> GND del Arduino (masa común)
    w((130, 68), (130, 57), (338, 57), (338, ay0 - 4))
    txt(262, 60, "− (en los capacitores) → GND del Arduino", fs=6.6, c="#333")

    # ---------------- tabla de cables
    tx, ty = 300, 140
    txt(tx, ty, "CABLES DEL BRAZO", fs=8.5, wgt="bold")
    filas = [("1", "+5V → LDR izq.", "5V"), ("2", "LDR izq. → R1", "R1 4,7k → GND"), ("3", "señal LDR izq.", "A0"),
             ("4", "+5V → LDR der.", "5V"), ("5", "LDR der. → R2", "R2 4,7k → GND"), ("6", "señal LDR der.", "A1"),
             ("7", "LED izq. (+)", "R_LED1 → 5V"), ("8", "LED izq. (−)", "GND"), ("9", "LED der. (+)", "R_LED2 → 5V"),
             ("10", "LED der. (−)", "GND")]
    for i, (n, a, b) in enumerate(filas):
        y = ty - 6.5 - i * 5.6
        txt(tx + 2, y, n, fs=7, ha="center", wgt="bold"); txt(tx + 7, y, a, fs=7); txt(tx + 52, y, b, fs=7)
    # notas
    nx, ny = 132, 46
    notas = ["Símbolo de tierra = GND del Arduino. Del − de la alimentación del servo, en el punto de los capacitores, sale un cable al GND del Arduino (masa común).",
             "Divisor de cada LDR: 5V – LDR – nodo – 4,7 kΩ – GND. Del nodo salen 2 cables: a la R y a la entrada analógica.",
             "R_LED: el valor que tengas puesto (típico 150–220 Ω para LED blanco a 5 V ≈ 10–13 mA).",
             "Firmware v2: LDR izq. = A0, der. = A1 (si están al revés la calibración lo detecta sola).",
             "Con los LED en 5V fijos, los parámetros ledi/ledd no actúan; para regular brillo, pasar el + de cada LED a D5/D6."]
    for i, n in enumerate(notas):
        txt(22, ny - i * 6.3 + 6, n, fs=6.9)
    pdf.savefig(fig); fig.savefig(OUT + "hoja5_esquema.png", dpi=110); plt.close(fig)


if __name__ == "__main__":
    import os
    os.makedirs(OUT, exist_ok=True)
    with PdfPages(OUT + "Planos_Seguidor_SG90.pdf") as pdf:
        hoja_servo(pdf)
        hoja_soporte(pdf)
        hoja_brazo(pdf)
        hoja_conjunto(pdf)
        hoja_esquema(pdf)
