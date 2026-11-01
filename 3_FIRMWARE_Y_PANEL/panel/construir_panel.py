"""Arma panel_v3.html: inserta la grabación de la simulación (demo.txt, comprimida) en el panel."""
import base64, gzip, os
aqui = os.path.dirname(os.path.abspath(__file__))
fuente = open(os.path.join(aqui, "panel_v3_fuente.html"), encoding="utf-8").read()
demo = open(os.path.join(aqui, "..", "pruebas_simulacion", "demo.txt"), "rb").read()
b64 = base64.b64encode(gzip.compress(demo, 9)).decode()
open(os.path.join(aqui, "..", "panel_v3.html"), "w", encoding="utf-8").write(fuente.replace("__DEMO__", b64))
print("panel_v3.html:", round(len(fuente) / 1024 + len(b64) / 1024), "KB")
