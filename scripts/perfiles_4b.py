#!/usr/bin/env python3
"""Parte 4B - Perfiles temporales cubico y quintico del acercamiento fino (recta pre-pick <-> pick).

Puntos: inicio, 3 puntos intermedios y final, EQUIESPACIADOS EN DISTANCIA sobre la recta.
Tiempos de los puntos: [0, alfa, 0.5, 1-alfa, 1] * T (simetricos).

  cubica   : spline cubico C2 por los 5 puntos, v = 0 en los extremos.
             La aceleracion NO es cero en los extremos -> salto de aceleracion al arrancar y al parar.
  quintica : spline quintico C4 por los 5 puntos, v = 0 y a = 0 en los extremos.

Para cada interpolacion y cada tramo se busca el alfa que da el menor T que cumple
|v| <= v_max y |a| <= a_max (restricciones de la consultora). v escala con 1/T y a con 1/T^2.

Tramos (interpretacion a confirmar con el profesor):
  rojo (ida)     pre-pick -> pick   v_max = 0.200 m/s  a_max = 0.300 m/s^2
  azul (retorno) pick -> pre-pick   v_max = 0.100 m/s  a_max = 0.020 m/s^2

Uso:  python3 scripts/perfiles_4b.py
Salida: resultados/4b_perfiles.json (puntos via con tiempos y perfiles muestreados) y graficas PNG.
"""
import json
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import CubicSpline, make_interp_spline

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "resultados")

PRE_PICK = np.array([0.55, -0.30, 0.40])
PICK = np.array([0.55, -0.30, 0.25])
TRAMOS = {
    "rojo_ida":     {"p0": PRE_PICK, "pf": PICK,     "v_max": 0.200, "a_max": 0.300},
    "azul_retorno": {"p0": PICK,     "pf": PRE_PICK, "v_max": 0.100, "a_max": 0.020},
}
N_INTERMEDIOS = 3
DT = 0.01          # muestreo de los perfiles [s]
MARGEN = 1.01      # 1 % de holgura sobre el T minimo


def spline(tipo, tau, s):
    if tipo == "cubica":
        return CubicSpline(tau, s, bc_type="clamped")
    return make_interp_spline(tau, s, k=5, bc_type=([(1, 0.0), (2, 0.0)], [(1, 0.0), (2, 0.0)]))


def picos(tipo, alfa, d):
    """Picos de velocidad y aceleracion con T = 1 s."""
    tau = np.array([0.0, alfa, 0.5, 1.0 - alfa, 1.0])
    f = spline(tipo, tau, np.linspace(0.0, d, N_INTERMEDIOS + 2))
    tt = np.linspace(0, 1, 4001)
    v, a = f(tt, 1), f(tt, 2)
    return float(np.max(np.abs(v))), float(np.max(np.abs(a))), float(np.min(v))


def disenar(tipo, d, v_max, a_max):
    mejor = None
    for alfa in np.arange(0.05, 0.4501, 0.005):
        v1, a1, vmin = picos(tipo, alfa, d)
        if vmin < -1e-9:              # descarta perfiles que retroceden
            continue
        T = max(v1 / v_max, np.sqrt(a1 / a_max)) * MARGEN
        if mejor is None or T < mejor[1]:
            mejor = (float(alfa), float(T))
    return mejor


def muestrear(tipo, alfa, T, p0, pf):
    d = float(np.linalg.norm(pf - p0))
    u = (pf - p0) / d
    t_k = np.array([0.0, alfa, 0.5, 1.0 - alfa, 1.0]) * T
    s_k = np.linspace(0.0, d, N_INTERMEDIOS + 2)
    f = spline(tipo, t_k, s_k)
    t = np.arange(0.0, T + 1e-9, DT)
    if t[-1] < T:
        t = np.append(t, T)
    s, v, a, j = f(t), f(t, 1), f(t, 2), f(t, 3)
    xyz = p0[None, :] + s[:, None] * u[None, :]
    return {
        "t_via": t_k.tolist(), "s_via": s_k.tolist(), "xyz_via": (p0[None, :] + s_k[:, None] * u[None, :]).tolist(),
        "t": t.tolist(), "s": s.tolist(), "v": v.tolist(), "a": a.tolist(), "j": j.tolist(), "xyz": xyz.tolist(),
        "T": T, "alfa": alfa, "v_pico": float(np.max(np.abs(v))), "a_pico": float(np.max(np.abs(a))),
        "a_inicio": float(a[0]), "a_final": float(a[-1]),
        "salto_a_inicio": float(abs(a[0])), "salto_a_final": float(abs(a[-1])),   # la aceleracion antes/despues del tramo es 0
        # jerk de la parte continua; en la cubica falta el impulso por el salto de aceleracion en los extremos
        "j_pico": float(np.max(np.abs(j))), "int_j2": float(np.trapz(j ** 2, t)),
    }


def main():
    os.makedirs(OUT, exist_ok=True)
    resultados = {}
    for nombre, tr in TRAMOS.items():
        d = float(np.linalg.norm(tr["pf"] - tr["p0"]))
        resultados[nombre] = {"p0": tr["p0"].tolist(), "pf": tr["pf"].tolist(), "d": d,
                              "v_max": tr["v_max"], "a_max": tr["a_max"], "perfiles": {}}
        print(f"\n== Tramo {nombre}: d = {d:.3f} m, v_max = {tr['v_max']} m/s, a_max = {tr['a_max']} m/s^2")
        for tipo in ("cubica", "quintica"):
            alfa, T = disenar(tipo, d, tr["v_max"], tr["a_max"])
            p = muestrear(tipo, alfa, T, tr["p0"], tr["pf"])
            resultados[nombre]["perfiles"][tipo] = p
            print(f"   {tipo:8s} alfa={alfa:.3f}  T={T:6.3f} s  t_via={np.round(p['t_via'], 3).tolist()}  "
                  f"v_pico={p['v_pico']:.4f} ({100*p['v_pico']/tr['v_max']:.0f}%)  a_pico={p['a_pico']:.4f} ({100*p['a_pico']/tr['a_max']:.0f}%)  "
                  f"a(0)={p['a_inicio']:+.4f}  a(T)={p['a_final']:+.4f}  j_pico={p['j_pico']:.3f}  int_j2={p['int_j2']:.4f}")
    json.dump(resultados, open(os.path.join(OUT, "4b_perfiles.json"), "w"), indent=1)

    # Graficas: una figura por tramo, filas s / v / a / j
    color = {"cubica": "#1f77b4", "quintica": "#d62728"}
    for nombre, r in resultados.items():
        fig, axs = plt.subplots(4, 1, figsize=(9, 10), sharex=True)
        for tipo, p in r["perfiles"].items():
            lab = f"{tipo} (T = {p['T']:.2f} s)"
            axs[0].plot(p["t"], p["s"], color=color[tipo], label=lab)
            axs[0].plot(p["t_via"], p["s_via"], "o", color=color[tipo], ms=6)
            axs[1].plot(p["t"], p["v"], color=color[tipo])
            # aceleracion: incluye el reposo antes y despues para que se vea el salto de la cubica
            axs[2].plot([-0.05 * p["T"], 0] + p["t"] + [p["T"], 1.05 * p["T"]], [0, 0] + p["a"] + [0, 0], color=color[tipo])
            axs[3].plot(p["t"], p["j"], color=color[tipo])
            if tipo == "cubica":  # salto de aceleracion en los extremos = impulso de jerk (no se ve en la curva)
                for tx in (0.0, p["T"]):
                    axs[3].axvline(tx, color=color[tipo], ls=":", lw=1.5)
                axs[3].annotate("cúbica: salto de a en t=0 y t=T\n→ jerk infinito (impulso)", xy=(0.0, 0), xytext=(0.12 * p["T"], 0.75),
                                textcoords=("data", "axes fraction"), color=color[tipo], fontsize=9,
                                arrowprops=dict(arrowstyle="->", color=color[tipo]))
        for ax, lim in ((axs[1], r["v_max"]), (axs[2], r["a_max"])):
            ax.axhline(lim, ls="--", color="k", lw=0.8)
            ax.axhline(-lim, ls="--", color="k", lw=0.8)
        axs[0].set_ylabel("s [m]"); axs[1].set_ylabel("v [m/s]"); axs[2].set_ylabel("a [m/s²]"); axs[3].set_ylabel("jerk [m/s³]")
        axs[3].set_xlabel("t [s]")
        axs[0].legend()
        for ax in axs:
            ax.grid(alpha=0.3)
        fig.suptitle(f"4B tramo {nombre.replace('_', ' ')}: d = {r['d']:.2f} m, "
                     f"v_max = {r['v_max']} m/s, a_max = {r['a_max']} m/s²  (o = puntos via)")
        fig.tight_layout()
        fig.savefig(os.path.join(OUT, f"4b_perfil_{nombre}.png"), dpi=100)
    print(f"\nGuardado en {OUT}: 4b_perfiles.json, 4b_perfil_rojo_ida.png, 4b_perfil_azul_retorno.png")


if __name__ == "__main__":
    main()
