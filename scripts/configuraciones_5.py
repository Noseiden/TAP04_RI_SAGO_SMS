#!/usr/bin/env python3
"""Parte 5 - Configuraciones donde se compara el Jacobiano analitico con el de MoveIt.

200 configuraciones aleatorias dentro de los limites del URDF + todos los puntos planeados de 4B y 4D
(resultados/4_ciclo.json). Escribe resultados/q_eval.csv (6 angulos por fila) y q_eval_etiquetas.csv.
Uso:  python3 scripts/configuraciones_5.py
"""
import json
import os
import subprocess
import xml.etree.ElementTree as ET

import numpy as np

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "resultados")

xacro = subprocess.run(
    ["bash", "-c", "xacro $(ros2 pkg prefix kr10_r1100_2_description)/share/kr10_r1100_2_description/urdf/kr10_r1100_2.urdf.xacro"],
    capture_output=True, text=True, check=True).stdout
lim = {j.get("name"): (float(j.find("limit").get("lower")), float(j.find("limit").get("upper")))
       for j in ET.fromstring(xacro).findall("joint") if j.get("type") == "revolute"}
lo = np.array([lim[f"joint_{i}"][0] for i in range(1, 7)])
hi = np.array([lim[f"joint_{i}"][1] for i in range(1, 7)])

filas = [("aleatoria", k, q) for k, q in enumerate(np.random.default_rng(5).uniform(lo, hi, (200, 6)))]
for nombre, r in json.load(open(os.path.join(OUT, "4_ciclo.json")))["rectas"].items():
    filas += [(nombre, k, np.array(q)) for k, q in enumerate(r["q"])]

with open(os.path.join(OUT, "q_eval.csv"), "w") as f:
    f.writelines(",".join(f"{v:.12f}" for v in q) + "\n" for _, _, q in filas)
with open(os.path.join(OUT, "q_eval_etiquetas.csv"), "w") as f:
    f.writelines(f"{n},{k}\n" for n, k, _ in filas)
print(f"{len(filas)} configuraciones en resultados/q_eval.csv")
