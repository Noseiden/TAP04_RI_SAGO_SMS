#!/usr/bin/env python3
"""Parte 4A - HOME -> pre-pick con evasion del poste: comparacion de planeadores OMPL.

Para cada planeador se repite la planeacion N veces (los planeadores de muestreo son aleatorios)
con el mismo inicio (HOME), la misma meta articular (IK de pre-pick) y la misma escena.

Metricas por intento:
  t_plan    tiempo de planeacion [s] (lo reporta MoveIt)
  L_q       longitud del camino en espacio articular: sum ||q_{k+1} - q_k||  [rad]
  L_tool    longitud del recorrido de tool0 en el espacio: sum ||p_{k+1} - p_k||  [m]  (p por DH)
  giro      suavidad geometrica: suma de los angulos entre segmentos consecutivos del camino
            articular, remuestreado a 50 puntos equiespaciados [rad]. Menor = mas suave.
  T_ejec    duracion de la trayectoria despues de la parametrizacion temporal (TOTG) [s]
  a_max     aceleracion articular maxima de esa trayectoria [rad/s^2]

Requiere move_group corriendo y la escena aplicada (scripts/escena.py).
Uso:  python3 scripts/comparar_planeadores_4a.py [--n 15] [--t 10] [--planeadores RRTConnect RRTstar]
"""
import argparse
import csv
import json
import math
import os
import sys

import numpy as np
import rclpy
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState

J = [f"joint_{i}" for i in range(1, 7)]
HOME = [0.0, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]
# IK de pre-pick (0.55, -0.30, 0.40), herramienta hacia abajo; obtenida con /compute_ik (KDL)
PRE_PICK_DEG = [28.61, -60.69, 114.06, 0.0, 36.64, 0.0]
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "resultados")


# ---------- cinematica directa DH (modificado / Craig), verificada contra el URDF ----------
def T_DH(a, al, s, th):
    ct, st, ca, sa = math.cos(th), math.sin(th), math.cos(al), math.sin(al)
    return np.array([[ct, -st, 0, a], [ca * st, ca * ct, -sa, -s * sa], [sa * st, sa * ct, ca, s * ca], [0, 0, 0, 1]])


def fk_tool(q):
    p = math.pi
    rows = [(0, 0, 0.400, -q[0]), (0.025, -p / 2, 0, q[1]), (0.560, 0, 0, q[2] - p / 2),
            (0.025, -p / 2, 0.515, -q[3]), (0, p / 2, 0, q[4]), (0, -p / 2, 0.090, p - q[5])]
    T = np.eye(4)
    for r in rows:
        T = T @ T_DH(*r)
    return T[:3, 3]


# ---------- metricas ----------
def remuestrear(Q, m=50):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))]
    if d[-1] < 1e-9:
        return Q
    s = np.linspace(0, d[-1], m)
    return np.column_stack([np.interp(s, d, Q[:, j]) for j in range(Q.shape[1])])


def giro_total(Q):
    V = np.diff(remuestrear(Q), axis=0)
    ang = 0.0
    for a, b in zip(V[:-1], V[1:]):
        na, nb = np.linalg.norm(a), np.linalg.norm(b)
        if na > 1e-9 and nb > 1e-9:
            ang += math.acos(max(-1.0, min(1.0, float(a @ b) / (na * nb))))
    return ang


def metricas(traj):
    pts = traj.joint_trajectory.points
    Q = np.array([p.positions for p in pts])
    P = np.array([fk_tool(q) for q in Q])
    A = np.array([p.accelerations for p in pts]) if pts and len(pts[0].accelerations) else np.zeros_like(Q)
    t = pts[-1].time_from_start
    return {
        "L_q": float(np.sum(np.linalg.norm(np.diff(Q, axis=0), axis=1))),
        "L_tool": float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))),
        "giro": giro_total(Q),
        "T_ejec": t.sec + t.nanosec * 1e-9,
        "a_max": float(np.max(np.abs(A))),
        "n_puntos": len(pts),
    }, Q, P


class Comparador(Node):
    def __init__(self):
        super().__init__("parte4a_comparar_planeadores")
        self.plan = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        for c, name in ((self.plan, "/plan_kinematic_path"), (self.valid, "/check_state_validity")):
            if not c.wait_for_service(timeout_sec=15.0):
                sys.exit(f"No aparece {name}: ¿esta corriendo move_group?")

    def call(self, cli, req, timeout):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def estado_valido(self, q):
        req = GetStateValidity.Request()
        req.group_name = "manipulator"
        req.robot_state = RobotState(joint_state=JointState(name=J, position=[float(v) for v in q]))
        res = self.call(self.valid, req, 5.0)
        return res.valid, sorted({f"{c.contact_body_1}-{c.contact_body_2}" for c in res.contacts})

    def planear(self, planner, q_goal, t_max):
        req = GetMotionPlan.Request()
        r = req.motion_plan_request
        r.group_name = "manipulator"
        r.pipeline_id = "ompl"
        r.planner_id = planner
        r.num_planning_attempts = 1
        r.allowed_planning_time = t_max
        r.max_velocity_scaling_factor = 0.5
        r.max_acceleration_scaling_factor = 0.5
        r.start_state = RobotState(joint_state=JointState(name=J, position=HOME))
        c = Constraints()
        for j, v in zip(J, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints = [c]
        res = self.call(self.plan, req, t_max + 20.0).motion_plan_response
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=15)
    ap.add_argument("--t", type=float, default=10.0, help="allowed_planning_time [s]")
    ap.add_argument("--planeadores", nargs="+", default=["RRTConnect", "RRTstar"])
    args = ap.parse_args()
    os.makedirs(OUT, exist_ok=True)
    q_goal = [math.radians(v) for v in PRE_PICK_DEG]

    rclpy.init()
    node = Comparador()

    # 1) El obstaculo si bloquea el camino directo: estados de la interpolacion articular recta
    print("== Camino articular recto HOME -> pre-pick (sin planeador)")
    choques = []
    for s in np.linspace(0, 1, 41):
        q = (1 - s) * np.array(HOME) + s * np.array(q_goal)
        ok, contactos = node.estado_valido(q)
        if not ok:
            choques.append((round(float(s), 3), contactos))
    for extremo, q in (("HOME", HOME), ("pre-pick", q_goal)):
        ok, contactos = node.estado_valido(q)
        print(f"   {extremo:8s} valido={ok} {contactos if contactos else ''}")
    print(f"   {len(choques)}/41 estados intermedios en colision"
          + (f", s en [{choques[0][0]}, {choques[-1][0]}], contactos: {sorted({c for _, cs in choques for c in cs})}" if choques else ""))

    # 2) Planeadores
    filas, resumen, mejores = [], {}, {}
    for planner in args.planeadores:
        print(f"\n== {planner}: {args.n} intentos, allowed_planning_time = {args.t} s")
        intentos = []
        for k in range(args.n):
            res = node.planear(planner, q_goal, args.t)
            fila = {"planeador": planner, "intento": k + 1, "exito": res.error_code.val == 1,
                    "t_plan": res.planning_time}
            if fila["exito"]:
                m, Q, P = metricas(res.trajectory)
                fila.update(m)
                intentos.append((fila, res.trajectory, Q, P))
                print(f"   {k+1:2d}  t={res.planning_time:6.3f}s  L_q={m['L_q']:.3f}  L_tool={m['L_tool']:.3f}  "
                      f"giro={m['giro']:.3f}  T={m['T_ejec']:.2f}s  a_max={m['a_max']:.2f}", flush=True)
            else:
                print(f"   {k+1:2d}  FALLO codigo {res.error_code.val}", flush=True)
            filas.append(fila)
        ok = [f for f, *_ in intentos]
        est = {"exitos": len(ok), "intentos": args.n}
        for key in ("t_plan", "L_q", "L_tool", "giro", "T_ejec", "a_max"):
            v = np.array([f[key] for f in ok]) if ok else np.array([np.nan])
            est[key] = {"media": float(np.mean(v)), "desv": float(np.std(v)), "min": float(np.min(v)), "max": float(np.max(v))}
        resumen[planner] = est
        if intentos:  # intento representativo: el de L_q mediana
            orden = sorted(intentos, key=lambda x: x[0]["L_q"])
            fila, traj, Q, P = orden[len(orden) // 2]
            mejores[planner] = {"intento": fila["intento"], "Q": Q.tolist(), "P": P.tolist(),
                                "t": [p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in traj.joint_trajectory.points]}

    # 3) Guardar
    with open(os.path.join(OUT, "4a_intentos.csv"), "w", newline="") as f:
        cols = ["planeador", "intento", "exito", "t_plan", "L_q", "L_tool", "giro", "T_ejec", "a_max", "n_puntos"]
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for fila in filas:
            w.writerow({c: fila.get(c, "") for c in cols})
    with open(os.path.join(OUT, "4a_resumen.json"), "w") as f:
        json.dump({"inicio_home_rad": HOME, "meta_pre_pick_deg": PRE_PICK_DEG, "allowed_planning_time": args.t,
                   "choques_camino_recto": choques, "resumen": resumen, "representativos": mejores}, f, indent=1)

    print("\n== Resumen (media ± desviacion)")
    print(f"   {'planeador':12s} {'exito':>6s} {'t_plan [s]':>16s} {'L_q [rad]':>14s} {'L_tool [m]':>14s} {'giro [rad]':>14s} {'T_ejec [s]':>14s}")
    for p, e in resumen.items():
        fmt = lambda k: f"{e[k]['media']:.3f}±{e[k]['desv']:.3f}"
        print(f"   {p:12s} {e['exitos']:>3d}/{e['intentos']:<2d} {fmt('t_plan'):>16s} {fmt('L_q'):>14s} {fmt('L_tool'):>14s} {fmt('giro'):>14s} {fmt('T_ejec'):>14s}")
    print(f"\nGuardado en {OUT}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
