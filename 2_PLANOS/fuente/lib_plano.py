"""Utilidades para dibujar planos técnicos A3 con matplotlib (unidades en mm)."""
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon, Rectangle, Circle, PathPatch, Arc
from matplotlib.path import Path

MM = 1 / 25.4
W, H = 420.0, 297.0
INK = "#111111"
THIN = 0.35
VIS = 0.9
FONT = "DejaVu Sans"
plt.rcParams["font.family"] = FONT
plt.rcParams["hatch.linewidth"] = 0.35


def num(v, d=1):
    v = round(v + (1e-6 if v >= 0 else -1e-6), d)      # redondeo hacia arriba en ,x5
    s = f"{v:.{d}f}".replace(".", ",")
    if "," in s:
        s = s.rstrip("0").rstrip(",")
    return s


def hoja(titulo, n, total, escala, sub=""):
    fig = plt.figure(figsize=(W * MM, H * MM))
    fr = fig.add_axes([0, 0, 1, 1])
    fr.set_xlim(0, W); fr.set_ylim(0, H); fr.axis("off")
    fr.add_patch(Rectangle((10, 10), W - 20, H - 20, fill=False, lw=1.4, color=INK))
    # rótulo
    x0, y0, w, h = 250, 10, 160, 38
    fr.add_patch(Rectangle((x0, y0), w, h, fill=False, lw=1.0, color=INK))
    for yy in (y0 + 26, y0 + 13):
        fr.plot([x0, x0 + w], [yy, yy], lw=0.6, color=INK)
    fr.plot([x0 + 105, x0 + 105], [y0, y0 + 26], lw=0.6, color=INK)
    fr.text(x0 + 3, y0 + 32, titulo, fontsize=12.5, weight="bold", va="center")
    fr.text(x0 + 3, y0 + 19.5, "Seguidor de línea estático · servo SG90 · brazo L = 120", fontsize=7.5, va="center")
    fr.text(x0 + 3, y0 + 6.5, sub or "Medidas en mm · PLA negro · tolerancia general ±0,2", fontsize=7.5, va="center")
    fr.text(x0 + 108, y0 + 19.5, f"Escala {escala}", fontsize=8, va="center")
    fr.text(x0 + 108, y0 + 6.5, f"Hoja {n} de {total} · 01/11/2026", fontsize=8, va="center")
    return fig, fr


class Vista:
    """Ejes con escala real: (x0,y0,w,h) en mm de papel, centro (cx,cy) en mm de pieza."""
    def __init__(self, fig, x0, y0, w, h, cx, cy, esc, titulo=None, fr=None):
        self.esc = esc
        self.ax = fig.add_axes([x0 / W, y0 / H, w / W, h / H])
        self.ax.set_xlim(cx - w / 2 / esc, cx + w / 2 / esc)
        self.ax.set_ylim(cy - h / 2 / esc, cy + h / 2 / esc)
        self.ax.set_aspect("equal"); self.ax.axis("off")
        if titulo and fr is not None:
            fr.text(x0 + w / 2, y0 - 4, titulo, ha="center", va="top", fontsize=9, weight="bold")

    def p(self, mm_papel):          # mm de papel -> unidades de pieza
        return mm_papel / self.esc

    # ---------------------------------------------------------------- geometría
    def poligonos(self, polys, lw=VIS, fc="none", hatch=None, ec=INK, z=2, alpha=1.0):
        verts, codes = [], []
        for p in polys:
            p = np.asarray(p)
            if len(p) < 3:
                continue
            verts += list(p) + [p[0]]
            codes += [Path.MOVETO] + [Path.LINETO] * (len(p) - 1) + [Path.CLOSEPOLY]
        if not verts:
            return
        pp = PathPatch(Path(verts, codes), fc=fc, ec=ec, lw=lw, hatch=hatch, zorder=z, alpha=alpha)
        self.ax.add_patch(pp)

    def linea(self, xs, ys, lw=VIS, ls="-", c=INK, z=3):
        self.ax.plot(xs, ys, lw=lw, ls=ls, color=c, zorder=z, solid_capstyle="butt")

    def rect(self, x0, y0, w, h, lw=VIS, fc="none", ls="-", hatch=None, z=2, ec=INK):
        self.ax.add_patch(Rectangle((x0, y0), w, h, fill=fc != "none", fc=fc, ec=ec, lw=lw, ls=ls, hatch=hatch, zorder=z))

    def circulo(self, x, y, r, lw=VIS, fc="none", ls="-", z=2, ec=INK):
        self.ax.add_patch(Circle((x, y), r, fill=fc != "none", fc=fc, ec=ec, lw=lw, ls=ls, zorder=z))

    def eje(self, x0, y0, x1, y1, ext=3):
        d = np.array([x1 - x0, y1 - y0]); L = np.linalg.norm(d); u = d / L; e = self.p(ext)
        self.ax.plot([x0 - u[0] * e, x1 + u[0] * e], [y0 - u[1] * e, y1 + u[1] * e], lw=0.35, color=INK,
                     ls=(0, (12, 3, 2, 3)), zorder=1)

    def cruz(self, x, y, r=4):
        e = self.p(r)
        self.eje(x - e, y, x + e, y, 0); self.eje(x, y - e, x, y + e, 0)

    def texto(self, x, y, t, fs=7.5, ha="center", va="center", rot=0, w="normal", c=INK, bg=False):
        kw = dict(bbox=dict(fc="white", ec="none", pad=0.6)) if bg else {}
        self.ax.text(x, y, t, fontsize=fs, ha=ha, va=va, rotation=rot, weight=w, color=c, zorder=6, **kw)

    # ---------------------------------------------------------------- cotas
    def cota(self, p1, p2, off, txt=None, d=1, fs=7.2, pref=""):
        p1 = np.array(p1, float); p2 = np.array(p2, float)
        v = p2 - p1; L = np.linalg.norm(v); u = v / L; n = np.array([-u[1], u[0]])
        sg = 1 if off >= 0 else -1
        q1 = p1 + n * off; q2 = p2 + n * off
        g, e = self.p(1.0), self.p(1.6)
        for a, b in ((p1, q1), (p2, q2)):
            self.ax.plot([a[0] + n[0] * g * sg, b[0] + n[0] * e * sg], [a[1] + n[1] * g * sg, b[1] + n[1] * e * sg],
                         lw=THIN, color=INK, zorder=4)
        self.ax.annotate("", xy=q2, xytext=q1, zorder=4,
                         arrowprops=dict(arrowstyle="<|-|>,head_length=0.45,head_width=0.16", lw=THIN, color=INK,
                                         shrinkA=0, shrinkB=0))
        ang = math.degrees(math.atan2(u[1], u[0]))
        if ang > 90.1 or ang < -89.9:
            ang += 180
        t = txt if txt is not None else pref + num(L, d)
        m = (q1 + q2) / 2 + n * sg * self.p(1.9)
        self.ax.text(m[0], m[1], t, fontsize=fs, ha="center", va="center", rotation=ang, zorder=6,
                     bbox=dict(fc="white", ec="none", pad=0.25))

    def cotah(self, x1, x2, y, yref1=None, yref2=None, txt=None, d=1):
        """Cota horizontal a la altura y, con líneas de referencia desde yref."""
        self._hv(x1, x2, y, yref1, yref2, txt, d, True)

    def cotav(self, y1, y2, x, xref1=None, xref2=None, txt=None, d=1):
        self._hv(y1, y2, x, xref1, xref2, txt, d, False)

    def _hv(self, a1, a2, pos, r1, r2, txt, d, horiz):
        r1 = pos if r1 is None else r1; r2 = pos if r2 is None else r2
        g, e = self.p(1.0), self.p(1.6)
        def lin(a, r):
            sg = 1 if pos >= r else -1
            if abs(pos - r) < 1e-9:
                return
            if horiz:
                self.ax.plot([a, a], [r + g * sg, pos + e * sg], lw=THIN, color=INK, zorder=4)
            else:
                self.ax.plot([r + g * sg, pos + e * sg], [a, a], lw=THIN, color=INK, zorder=4)
        lin(a1, r1); lin(a2, r2)
        p1, p2 = ((a1, pos), (a2, pos)) if horiz else ((pos, a1), (pos, a2))
        self.ax.annotate("", xy=p2, xytext=p1, zorder=4,
                         arrowprops=dict(arrowstyle="<|-|>,head_length=0.45,head_width=0.16", lw=THIN, color=INK,
                                         shrinkA=0, shrinkB=0))
        t = txt if txt is not None else num(abs(a2 - a1), d)
        o = self.p(1.9)
        if horiz:
            sg = 1 if pos >= min(r1, r2) else -1
            self.ax.text((a1 + a2) / 2, pos + o * sg, t, fontsize=7.2, ha="center", va="center", zorder=6,
                         bbox=dict(fc="white", ec="none", pad=0.25))
        else:
            sg = 1 if pos >= min(r1, r2) else -1
            self.ax.text(pos + o * sg, (a1 + a2) / 2, t, fontsize=7.2, ha="center", va="center", rotation=90, zorder=6,
                         bbox=dict(fc="white", ec="none", pad=0.25))

    def nota(self, x, y, tx, ty, t, fs=7.2, ha=None):
        """Flecha de referencia desde (x,y) hasta el texto en (tx,ty)."""
        ha = ha or ("left" if tx >= x else "right")
        self.ax.annotate(t, xy=(x, y), xytext=(tx, ty), fontsize=fs, ha=ha, va="center", zorder=6,
                         arrowprops=dict(arrowstyle="-|>,head_length=0.4,head_width=0.15", lw=THIN, color=INK,
                                         shrinkA=1, shrinkB=0),
                         bbox=dict(fc="white", ec="none", pad=0.3))

    def diam(self, x, y, r, ang_deg, largo, t=None, fs=7.2):
        a = math.radians(ang_deg)
        p = (x + r * math.cos(a), y + r * math.sin(a))
        q = (x + (r + self.p(largo)) * math.cos(a), y + (r + self.p(largo)) * math.sin(a))
        self.nota(p[0], p[1], q[0], q[1], t or "Ø" + num(2 * r), fs)

    def angulo(self, cx, cy, r, a0, a1, t, fs=7.2):
        self.ax.add_patch(Arc((cx, cy), 2 * r, 2 * r, theta1=a0, theta2=a1, lw=THIN, color=INK, zorder=4))
        am = math.radians((a0 + a1) / 2)
        rr = r + self.p(3.5)
        self.texto(cx + rr * math.cos(am), cy + rr * math.sin(am), t, fs=fs, bg=True)


def corte_letras(v, x0, y0, x1, y1, letra):
    """Línea de corte con flechas y letra (en coordenadas de la vista)."""
    v.ax.plot([x0, x1], [y0, y1], lw=0.5, color=INK, ls=(0, (12, 3, 2, 3)), zorder=5)
    d = np.array([x1 - x0, y1 - y0]); u = d / np.linalg.norm(d); n = np.array([u[1], -u[0]])
    for (x, y) in ((x0, y0), (x1, y1)):
        v.ax.plot([x, x], [y, y], marker="", zorder=5)
        a = v.p(4)
        v.ax.annotate("", xy=(x + n[0] * a, y + n[1] * a), xytext=(x, y),
                      arrowprops=dict(arrowstyle="-|>,head_length=0.5,head_width=0.2", lw=0.9, color=INK), zorder=5)
        v.texto(x - u[0] * v.p(3.5) * (1 if (x, y) == (x0, y0) else -1), y - u[1] * v.p(3.5) * (1 if (x, y) == (x0, y0) else -1),
                letra, fs=9, w="bold")
