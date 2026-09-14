#!/usr/bin/env python3
"""Parte 4 - Ciclo completo pick-and-place (4A -> 4B -> 4C -> 4D) en MoveIt2.

  0. Preparacion: escena limpia (pieza sobre mesa_pick, nada pegado a la herramienta) y robot en HOME.
  1. 4A  HOME -> pre-pick            RRTConnect (rodea el poste)
  2. 4B  pre-pick -> pick            recta, perfil quintico tramo rojo (ida)
  3.     agarrar: la pieza pasa a ser un objeto pegado a tool0
  4. 4B  pick -> pre-pick            recta, perfil quintico tramo azul (retorno), con la pieza
  5. 4C  pre-pick -> pre-place       RRTConnect con la pieza (rodea el poste)
  6. 4D  pre-place -> place          recta, perfil quintico tramo rojo (ida)
  7.     soltar: la pieza vuelve al mundo, sobre mesa_place
  8. 4D  place -> pre-place          recta, perfil quintico tramo azul (retorno)
  9.     pre-place -> HOME           RRTConnect

Graba /joint_states de todo el ciclo y guarda resultados/4_ciclo.json + graficas.
Requiere move_group corriendo y resultados/4b_perfiles.json.
Uso:  python3 scripts/ciclo_4.py [--perfil quintica]
"""
import argparse
import json
import math
import os
import sys
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject, PlanningScene, PlanningSceneComponents
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene

import escena
from cartesiano_4b import HOME, J, OUT, Cartesiano, fk_tool, reparametrizar, tool_vel_acc

PICK = np.array([0.55, -0.30, 0.25])
PLACE = np.array([0.45, 0.40, 0.35])
ALTURA_PRE = 0.15


def quat_abajo(p):
    psi = math.atan2(p[1], p[0])
    return [-math.sin(psi / 2), math.cos(psi / 2), 0.0, 0.0]


def perfil_en_recta(perfil, p0, pf):
    """El perfil s(t) es el mismo para cualquier recta de la misma longitud: solo cambian los puntos via."""
    d = float(np.linalg.norm(pf - p0))
    u = (pf - p0) / d
    nuevo = dict(perfil)
    nuevo["xyz_via"] = (p0[None, :] + np.array(perfil["s_via"])[:, None] * u[None, :]).tolist()
    return nuevo


class Ciclo(Cartesiano):
    def __init__(self):
        super().__init__()
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.get_scene = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.apply.wait_for_service(timeout_sec=15)
        self.get_scene.wait_for_service(timeout_sec=15)
        self.log = []          # (t, q, qd) de todo el ciclo
        self.rectas = {}       # trayectorias planeadas de los tramos rectos (para la parte 5)
        self.tramos = []       # (nombre, t_inicio, t_fin)
        self.t_js = None

    def _js(self, m):
        super()._js(m)
        if all(j in m.name for j in J):
            self.t_js = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            vel = [m.velocity[m.name.index(j)] for j in J] if len(m.velocity) == len(m.name) else [float("nan")] * 6
            self.log.append((self.t_js, [m.position[m.name.index(j)] for j in J], vel))

    def aplicar(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        return self.esperar(self.apply.call_async(ApplyPlanningScene.Request(scene=scene)), 10).success

    def pieza_pegada(self, pegar):
        aco = AttachedCollisionObject(link_name="tool0", touch_links=["tool0", "link_6"])
        aco.object.id = "pieza"
        aco.object.header.frame_id = "tool0"
        aco.object.operation = CollisionObject.ADD if pegar else CollisionObject.REMOVE
        scene = PlanningScene()
        scene.robot_state.attached_collision_objects = [aco]
        return self.aplicar(scene)

    def escena_inicial(self):
        self.pieza_pegada(False)                          # por si quedo pegada de una corrida anterior
        scene = PlanningScene()
        scene.world.collision_objects = [escena.objeto(n, *v, CollisionObject.ADD) for n, v in escena.OBJETOS.items()]
        return self.aplicar(scene)

    def pose_pieza(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_GEOMETRY | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        sc = self.esperar(self.get_scene.call_async(req), 10).scene
        for co in sc.world.collision_objects:
            if co.id == "pieza":
                p = co.pose.position if (co.pose.position.x or co.pose.position.y or co.pose.position.z) else co.primitive_poses[0].position
                return "mundo", [p.x, p.y, p.z]
        if any(a.object.id == "pieza" for a in sc.robot_state.attached_collision_objects):
            return "pegada a tool0", None
        return "no existe", None

    def tramo(self, nombre, fn):
        rclpy.spin_once(self, timeout_sec=0.05)
        t0 = self.t_js
        ok, info = fn()
        rclpy.spin_once(self, timeout_sec=0.05)
        self.tramos.append((nombre, t0, self.t_js))
        print(f"   [{'OK' if ok else 'FALLO'}] {nombre} ({self.t_js - t0:.2f} s) {info}", flush=True)
        if not ok:
            sys.exit(f"Se detiene el ciclo en: {nombre}")

    def recta(self, nombre, q_ini, perfil, p0, pf):
        per = perfil_en_recta(perfil, p0, pf)
        sol = self.camino_cartesiano(q_ini, per["xyz_via"], quat_abajo(pf))
        if sol.fraction < 0.999:
            return False, f"computeCartesianPath fraction = {sol.fraction:.3f}", None
        tr = reparametrizar(sol.solution, per, p0, pf)
        vt, at = tool_vel_acc(tr["q"], tr["qd"], tr["qdd"])
        rclpy.spin_once(self, timeout_sec=0.01)
        self.rectas[nombre] = {"t_inicio_js": self.t_js, "p0": p0.tolist(), "pf": pf.tolist(),
                               "t": tr["t"].tolist(), "q": tr["q"].tolist(), "qd": tr["qd"].tolist(), "qdd": tr["qdd"].tolist(),
                               "v_perfil": perfil["v"], "a_perfil": perfil["a"]}
        code, _ = self.ejecutar(tr["t"], tr["q"], tr["qd"], tr["qdd"])
        return code == 1, f"v_tool_max = {vt.max():.3f} m/s, a_tool_max = {at.max():.3f} m/s^2, codigo {code}", tr["q"][-1].tolist()


def graficar(data):
    t, P, tramos = np.array(data["t"]), np.array(data["p_tool"]), data["tramos"]
    v = np.linalg.norm(np.gradient(P, t, axis=0), axis=1)
    tipo = lambda n: ("planeador (RRTConnect)", "#fcbba1") if "RRTConnect" in n else \
        ("recta ida (rojo)", "#c6dbef") if "ida" in n else ("recta retorno (azul)", "#c7e9c0") if "retorno" in n else (None, None)
    etiqueta = {"4A": "4A", "4B  pre": "4B ida", "4B  pick": "4B ret.", "4C": "4C", "4D  pre": "4D ida", "4D  place": "4D ret.", "    pre-place": "HOME"}
    fig, axs = plt.subplots(2, 1, figsize=(13, 6.5), sharex=True)
    usados = set()
    for n, a, b in tramos:
        lab, col = tipo(n)
        if col is None:
            for ax in axs:
                ax.axvline(a, color="k", ls="--", lw=0.8)
            axs[0].text(a, 1.08, "agarra" if "agarrar" in n else "suelta", ha="center", fontsize=8)
            continue
        for ax in axs:
            ax.axvspan(a, b, color=col, lw=0, label=None if lab in usados else lab)
        usados.add(lab)
        corto = next((v_ for k, v_ in etiqueta.items() if n.startswith(k)), "")
        axs[1].text((a + b) / 2, 1.9, corto, ha="center", fontsize=8)
    for k, lab in enumerate("xyz"):
        axs[0].plot(t, P[:, k], lw=1.5, label=f"{lab} tool0")
    axs[0].set_ylabel("posicion [m]"); axs[0].set_ylim(-0.7, 1.15)
    axs[0].legend(loc="lower left", fontsize=8, ncol=6)
    axs[1].plot(t, v, color="k", lw=1)
    axs[1].axhline(0.2, ls=":", color="r"); axs[1].axhline(0.1, ls=":", color="b")
    axs[1].set_ylim(0, 2.05)
    axs[1].set_ylabel("|v tool0| [m/s]"); axs[1].set_xlabel("t [s]")
    for ax in axs:
        ax.grid(alpha=0.3)
    fig.suptitle(f"Ciclo pick-and-place completo, perfil {data['perfil']} (medido en /joint_states + DH): {tramos[-1][2]:.1f} s")
    fig.tight_layout(); fig.savefig(os.path.join(OUT, "4_ciclo.png"), dpi=100)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--perfil", default="quintica", choices=["cubica", "quintica"])
    args = ap.parse_args()
    perfiles = json.load(open(os.path.join(OUT, "4b_perfiles.json")))
    ida = perfiles["rojo_ida"]["perfiles"][args.perfil]
    ret = perfiles["azul_retorno"]["perfiles"][args.perfil]

    rclpy.init()
    node = Ciclo()
    fin = time.time() + 15.0                       # la suscripcion puede tardar en conectarse (descubrimiento DDS)
    while node.t_js is None and time.time() < fin:
        rclpy.spin_once(node, timeout_sec=0.1)
    if node.t_js is None:
        sys.exit("No llegan mensajes de /joint_states")
    pre_pick, pre_place = PICK + [0, 0, ALTURA_PRE], PLACE + [0, 0, ALTURA_PRE]
    Q = {n: node.ik_exacta(p, quat_abajo(p)) for n, p in (("pre_pick", pre_pick), ("pre_place", pre_place))}
    if None in Q.values():
        sys.exit("IK de pre-pick / pre-place fallo")

    print("== 0. Preparacion")
    print("   escena inicial:", node.escena_inicial(), "| pieza:", node.pose_pieza())
    print("   robot a HOME: codigo", node.ir_a(HOME))

    print(f"\n== Ciclo pick-and-place (planeador RRTConnect, perfil {args.perfil})")
    node.log.clear()
    estado = {}
    node.tramo("4A  HOME -> pre-pick (RRTConnect)", lambda: (node.ir_a(Q["pre_pick"]) == 1, ""))

    def ida_pick():
        ok, info, q = node.recta("4B_ida", Q["pre_pick"], ida, pre_pick, PICK); estado["q"] = q; return ok, info
    node.tramo("4B  pre-pick -> pick (ida, rojo)", ida_pick)
    node.tramo("    agarrar pieza", lambda: (node.pieza_pegada(True), node.pose_pieza()))

    def ret_pick():
        ok, info, q = node.recta("4B_retorno", estado["q"], ret, PICK, pre_pick); return ok, info
    node.tramo("4B  pick -> pre-pick (retorno, azul, con pieza)", ret_pick)
    node.tramo("4C  pre-pick -> pre-place (RRTConnect, con pieza)", lambda: (node.ir_a(Q["pre_place"]) == 1, ""))

    def ida_place():
        ok, info, q = node.recta("4D_ida", Q["pre_place"], ida, pre_place, PLACE); estado["q"] = q; return ok, info
    node.tramo("4D  pre-place -> place (ida, rojo)", ida_place)
    node.tramo("    soltar pieza", lambda: (node.pieza_pegada(False), node.pose_pieza()))

    def ret_place():
        ok, info, q = node.recta("4D_retorno", estado["q"], ret, PLACE, pre_place); return ok, info
    node.tramo("4D  place -> pre-place (retorno, azul)", ret_place)
    node.tramo("    pre-place -> HOME (RRTConnect)", lambda: (node.ir_a(HOME) == 1, ""))

    donde, p = node.pose_pieza()
    esperado = [PLACE[0], PLACE[1], PLACE[2] - 0.01 - 0.02]      # tool0 1 cm sobre la pieza, centro 2 cm mas abajo
    err = None if p is None else 1000 * float(np.linalg.norm(np.array(p) - esperado))
    print(f"\n== Pieza al final: {donde} {np.round(p, 4).tolist() if p else ''} (esperado {np.round(esperado, 3).tolist()}, error {err:.2f} mm)")

    # ---- guardar ----
    t = np.array([s[0] for s in node.log]); Qlog = np.array([s[1] for s in node.log]); QDlog = np.array([s[2] for s in node.log])
    t0 = t[0]
    tramos = [(n, a - t0, b - t0) for n, a, b in node.tramos]
    P = np.array([fk_tool(q) for q in Qlog])
    v = np.linalg.norm(np.gradient(P, t, axis=0), axis=1)
    for r in node.rectas.values():
        r["t_inicio_js"] -= t0
    json.dump({"perfil": args.perfil, "tramos": tramos, "t": (t - t0).tolist(), "q": Qlog.tolist(), "qd": QDlog.tolist(),
               "p_tool": P.tolist(), "rectas": node.rectas,
               "pieza_final": p, "pieza_esperada": esperado, "error_pieza_mm": err}, open(os.path.join(OUT, "4_ciclo.json"), "w"))
    print("\n== Duracion por tramo")
    for n, a, b in tramos:
        print(f"   {b - a:6.2f} s  {n}")
    print(f"   {tramos[-1][2]:6.2f} s  TOTAL")

    graficar(json.load(open(os.path.join(OUT, "4_ciclo.json"))))
    print(f"\nGuardado en {OUT}: 4_ciclo.json, 4_ciclo.png")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
