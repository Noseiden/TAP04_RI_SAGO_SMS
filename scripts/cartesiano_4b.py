#!/usr/bin/env python3
"""Parte 4B - Acercamiento fino con computeCartesianPath + re-parametrizacion temporal (cubica vs quintica).

Para cada tramo (rojo ida: pre-pick -> pick, azul retorno: pick -> pre-pick) y cada perfil de 4b_perfiles.json:
  1. /compute_cartesian_path con los puntos via del perfil (herramienta hacia abajo, paso de 5 mm).
     MoveIt devuelve el camino articular con SU parametrizacion temporal (TOTG) -> se guarda como referencia.
  2. Re-parametrizacion temporal con el perfil: para cada punto del camino se calcula s (avance sobre la recta,
     con la cinematica directa DH) y se ajusta q(s) con un spline. Luego, con el perfil s(t):
        q(t) = q(s(t)),   qd = q'(s) sd,   qdd = q''(s) sd^2 + q'(s) sdd
  3. /execute_trajectory con la trayectoria re-parametrizada (posicion, velocidad y aceleracion cada 10 ms),
     grabando /joint_states para medir la velocidad real de tool0.

Requiere move_group corriendo, la escena (scripts/escena.py) y resultados/4b_perfiles.json (scripts/perfiles_4b.py).
Uso:  python3 scripts/cartesiano_4b.py
"""
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
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, RobotState, RobotTrajectory
from moveit_msgs.srv import GetCartesianPath, GetPositionIK
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from scipy.interpolate import CubicSpline
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

J = [f"joint_{i}" for i in range(1, 7)]
HOME = [0.0, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]
DT_CONTROL = 0.01   # periodo del controlador (update_rate = 100 Hz): un salto de aceleracion ocurre en un ciclo
OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "resultados")
PERFILES = ("cubica", "quintica")
COLOR = {"cubica": "#1f77b4", "quintica": "#d62728", "TOTG": "#7f7f7f"}


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


def tool_speed(t, Q):
    """Solo para datos medidos (joint_states): derivada numerica de la posicion de tool0."""
    P = np.array([fk_tool(q) for q in Q])
    V = np.gradient(P, t, axis=0)
    return P, np.linalg.norm(V, axis=1)


def jac_num(q, h=1e-6):
    """Jacobiano de posicion 3x6 por diferencias centrales de la DH (el analitico se hace en la parte 5)."""
    q = np.asarray(q, float)
    return np.column_stack([(fk_tool(q + h * e) - fk_tool(q - h * e)) / (2 * h) for e in np.eye(6)])


def tool_vel_acc(Q, Qd, Qdd, h=1e-6):
    """v = J qd,  a = J qdd + Jdot qd  (Jdot qd = derivada direccional de J a lo largo de qd)."""
    V, A = [], []
    for q, qd, qdd in zip(Q, Qd, Qdd):
        Jq = jac_num(q)
        Jdot_qd = (jac_num(q + h * qd) - jac_num(q - h * qd)) / (2 * h) @ qd
        V.append(Jq @ qd)
        A.append(Jq @ qdd + Jdot_qd)
    return np.linalg.norm(V, axis=1), np.linalg.norm(A, axis=1)


class Cartesiano(Node):
    def __init__(self):
        super().__init__("parte4b_cartesiano")
        self.cart = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.exe = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        if not self.cart.wait_for_service(timeout_sec=15) or not self.move.wait_for_server(timeout_sec=15) \
                or not self.exe.wait_for_server(timeout_sec=15):
            sys.exit("No aparecen los servicios/acciones de move_group")
        self.grabando = None
        self.ultimo = None
        self.create_subscription(JointState, "/joint_states", self._js, 100)

    def _js(self, m):
        if not all(j in m.name for j in J):
            return
        q = [m.position[m.name.index(j)] for j in J]
        self.ultimo = q
        if self.grabando is not None:
            self.grabando.append((m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, q))

    def esperar(self, fut, timeout):
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        return fut.result()

    def ir_a(self, q):
        goal = MoveGroup.Goal()
        r = goal.request
        r.group_name, r.pipeline_id, r.planner_id = "manipulator", "ompl", "RRTConnect"
        r.allowed_planning_time, r.num_planning_attempts = 5.0, 1
        r.max_velocity_scaling_factor = r.max_acceleration_scaling_factor = 0.5
        c = Constraints()
        for j, v in zip(J, q):
            c.joint_constraints.append(JointConstraint(joint_name=j, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        r.goal_constraints = [c]
        gh = self.esperar(self.move.send_goal_async(goal), 10)
        return self.esperar(gh.get_result_async(), 60).result.error_code.val

    def ik_exacta(self, p, quat):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name, r.ik_link_name, r.avoid_collisions = "manipulator", "tool0", False
        r.timeout.sec = 1
        seed = list(HOME); seed[0] = -math.atan2(p[1], p[0])
        r.robot_state = RobotState(joint_state=JointState(name=J, position=seed))
        r.pose_stamped = PoseStamped(); r.pose_stamped.header.frame_id = "base_link"
        w = r.pose_stamped.pose
        w.position.x, w.position.y, w.position.z = [float(c) for c in p]
        w.orientation.x, w.orientation.y, w.orientation.z, w.orientation.w = quat
        res = self.esperar(self.ik.call_async(req), 10)
        js = res.solution.joint_state
        return [js.position[js.name.index(j)] for j in J] if res.error_code.val == 1 else None

    def camino_cartesiano(self, q_inicio, xyz_via, quat):
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.start_state = RobotState(joint_state=JointState(name=J, position=[float(v) for v in q_inicio]))
        req.group_name, req.link_name = "manipulator", "tool0"
        for p in xyz_via[1:]:                       # el primer punto es el estado inicial
            w = Pose()
            w.position.x, w.position.y, w.position.z = [float(c) for c in p]
            w.orientation.x, w.orientation.y, w.orientation.z, w.orientation.w = quat
            req.waypoints.append(w)
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = req.max_acceleration_scaling_factor = 0.1
        return self.esperar(self.cart.call_async(req), 30)

    def ejecutar(self, t, Q, Qd, Qdd):
        jt = JointTrajectory(joint_names=J)
        for ti, q, qd, qdd in zip(t, Q, Qd, Qdd):
            pt = JointTrajectoryPoint(positions=[float(v) for v in q], velocities=[float(v) for v in qd],
                                      accelerations=[float(v) for v in qdd])
            pt.time_from_start.sec = int(ti)
            pt.time_from_start.nanosec = int(round((ti - int(ti)) * 1e9))
            jt.points.append(pt)
        goal = ExecuteTrajectory.Goal(trajectory=RobotTrajectory(joint_trajectory=jt))
        self.grabando = []
        gh = self.esperar(self.exe.send_goal_async(goal), 10)
        res = self.esperar(gh.get_result_async(), t[-1] + 30)
        fin = time.time() + 0.3
        while time.time() < fin:
            rclpy.spin_once(self, timeout_sec=0.02)
        muestras, self.grabando = self.grabando, None
        return res.result.error_code.val, muestras


def reparametrizar(sol, perfil, p0, pf):
    pts = sol.joint_trajectory.points
    Q = np.array([p.positions for p in pts])
    P = np.array([fk_tool(q) for q in Q])
    d = float(np.linalg.norm(pf - p0))
    u = (pf - p0) / d
    s = (P - p0) @ u
    desvio = float(np.max(np.linalg.norm((P - p0) - np.outer(s, u), axis=1)))   # distancia a la recta
    keep = np.r_[True, np.diff(s) > 1e-6]
    Qs = CubicSpline(s[keep], Q[keep], axis=0)
    t = np.array(perfil["t"])
    sp = np.clip(np.array(perfil["s"]), s[keep][0], s[keep][-1])
    v, a = np.array(perfil["v"]), np.array(perfil["a"])
    q, dq, ddq = Qs(sp), Qs(sp, 1), Qs(sp, 2)
    qd = dq * v[:, None]
    qdd = ddq * (v ** 2)[:, None] + dq * a[:, None]
    # referencia: tiempos que puso MoveIt (TOTG)
    t_totg = np.array([p.time_from_start.sec + p.time_from_start.nanosec * 1e-9 for p in pts])
    qdd_totg = np.array([p.accelerations for p in pts]) if len(pts[0].accelerations) else np.gradient(np.gradient(Q, t_totg, axis=0), t_totg, axis=0)
    return {"t": t, "q": q, "qd": qd, "qdd": qdd, "desvio_recta": desvio, "n_cart": len(pts),
            "t_totg": t_totg, "Q_totg": Q, "qdd_totg": qdd_totg}


def metricas(tr, v_lim, a_lim):
    t, qdd = tr["t"], tr["qdd"]
    jerk = np.gradient(qdd, t, axis=0)
    vt, at = tool_vel_acc(tr["q"], tr["qd"], tr["qdd"])
    _, vt_totg = tool_speed(tr["t_totg"], tr["Q_totg"])
    salto0, saltoT = float(np.linalg.norm(qdd[0])), float(np.linalg.norm(qdd[-1]))
    int_j2 = float(np.trapz(np.sum(jerk ** 2, axis=1), t))
    return {
        "T": float(t[-1]),
        "qdd_max_por_joint": np.max(np.abs(qdd), axis=0).round(4).tolist(),
        "qdd_max": float(np.max(np.abs(qdd))),
        "qdd_rms": float(np.sqrt(np.mean(qdd ** 2))),
        "salto_qdd_inicio": salto0,    # antes del tramo el robot esta quieto (qdd = 0)
        "salto_qdd_final": saltoT,
        "jerk_art_max_continuo": float(np.max(np.abs(jerk))),
        "jerk_art_salto": max(salto0, saltoT) / DT_CONTROL,   # el salto ocurre en un ciclo del controlador
        "int_jerk2_continuo": int_j2,
        "int_jerk2_con_saltos": int_j2 + (salto0 ** 2 + saltoT ** 2) / DT_CONTROL,
        "v_tool_max": float(np.max(vt)), "a_tool_max": float(np.max(at)),
        "cumple_v": bool(np.max(vt) <= v_lim * 1.001), "cumple_a": bool(np.max(at) <= a_lim * 1.001),
        "totg_T": float(tr["t_totg"][-1]), "totg_v_tool_max": float(np.max(vt_totg)),
        "totg_qdd_max": float(np.max(np.abs(tr["qdd_totg"]))),
    }


def alinear_ejecucion(c, umbral=1e-4):
    """Desplaza el tiempo medido para que el inicio del movimiento coincida con el del plan
    (MoveIt tarda unas decimas en arrancar la ejecucion; no es error de seguimiento)."""
    te, Qe = np.array(c["t_ejec"]), np.array(c["q_ejec"])
    tp, Qp = np.array(c["t"]), np.array(c["q"])
    ke = np.argmax(np.max(np.abs(Qe - Qe[0]), axis=1) > umbral)
    kp = np.argmax(np.max(np.abs(Qp - Qp[0]), axis=1) > umbral)
    return te - (te[ke] - tp[kp])


def graficas(salida, perfiles):
    for tramo, casos in salida.items():
        fig, axs = plt.subplots(2, 3, figsize=(14, 7), sharex=True)
        for tipo, c in casos.items():
            t, qdd = np.array(c["t"]), np.array(c["qdd"])
            for k, ax in enumerate(axs.flat):
                ax.plot(np.r_[-0.05 * t[-1], 0, t, t[-1], 1.05 * t[-1]], np.r_[0, 0, qdd[:, k], 0, 0], color=COLOR[tipo],
                        label=f"{tipo} (T={t[-1]:.2f} s)")
                ax.set_title(J[k]); ax.grid(alpha=0.3)
                if np.max(np.abs(qdd[:, k])) < 1e-6:
                    ax.set_ylim(-0.05, 0.05)
                    ax.text(0.5, 0.75, "≈ 0 (ruido numérico 1e-14)\nla recta es vertical y la orientación fija:\nsolo se mueven joint_2, joint_3 y joint_5",
                            transform=ax.transAxes, ha="center", fontsize=8)
        axs[0, 0].legend(fontsize=8)
        for ax in axs[1]:
            ax.set_xlabel("t [s]")
        for ax in axs[:, 0]:
            ax.set_ylabel("q̈ [rad/s²]")
        fig.suptitle(f"4B {tramo.replace('_', ' ')}: aceleracion articular re-parametrizada (reposo antes y despues del tramo)")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, f"4b_qdd_{tramo}.png"), dpi=100)

        fig, ax = plt.subplots(figsize=(9, 4.5))
        v_lim = perfiles[tramo]["v_max"]
        for tipo, c in casos.items():
            p = perfiles[tramo]["perfiles"][tipo]
            ax.plot(p["t"], np.abs(p["v"]), color=COLOR[tipo], lw=2, label=f"{tipo}: perfil")
            ax.plot(alinear_ejecucion(c), c["v_tool_ejec"], ".", ms=3, color=COLOR[tipo], alpha=0.6,
                    label=f"{tipo}: ejecutado (joint_states + DH, alineado al inicio del movimiento)")
        c = casos["quintica"]
        _, vt = tool_speed(np.array(c["t_totg"]), np.array(c["q_totg"]))
        ax.plot(c["t_totg"], vt, "--", color=COLOR["TOTG"], label="tiempos por defecto de MoveIt (TOTG)")
        ax.axhline(v_lim, ls=":", color="k", label=f"v_max = {v_lim} m/s")
        ax.set_xlim(-0.05 * c["t"][-1], 1.15 * max(casos["cubica"]["t"][-1], c["t"][-1]))
        ax.set_xlabel("t [s]"); ax.set_ylabel("|v tool0| [m/s]"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
        ax.set_title(f"4B {tramo.replace('_', ' ')}: velocidad de la herramienta")
        fig.tight_layout(); fig.savefig(os.path.join(OUT, f"4b_vtool_{tramo}.png"), dpi=100)


def main():
    perfiles = json.load(open(os.path.join(OUT, "4b_perfiles.json")))
    rclpy.init()
    node = Cartesiano()

    p_pre = np.array(perfiles["rojo_ida"]["p0"])
    psi0 = math.atan2(p_pre[1], p_pre[0])
    PRE_PICK_Q = node.ik_exacta(p_pre, [-math.sin(psi0 / 2), math.cos(psi0 / 2), 0.0, 0.0])
    print("== IK exacta de pre-pick [deg]:", np.round(np.degrees(PRE_PICK_Q), 4).tolist())
    print("== Llevando el robot a pre-pick (RRTConnect)")
    code = node.ir_a(PRE_PICK_Q)
    print("   codigo", code)
    if code != 1:
        sys.exit("No se pudo llegar a pre-pick")

    salida = {}
    q_actual = PRE_PICK_Q
    for tipo in PERFILES:
        for tramo in ("rojo_ida", "azul_retorno"):
            tr_def = perfiles[tramo]
            perfil = tr_def["perfiles"][tipo]
            p0, pf = np.array(tr_def["p0"]), np.array(tr_def["pf"])
            psi = math.atan2(p0[1], p0[0])
            quat = [-math.sin(psi / 2), math.cos(psi / 2), 0.0, 0.0]
            print(f"\n== {tramo} / {tipo}: T = {perfil['T']:.2f} s")
            sol = node.camino_cartesiano(q_actual, perfil["xyz_via"], quat)
            print(f"   computeCartesianPath: fraction = {sol.fraction:.3f}, puntos = {len(sol.solution.joint_trajectory.points)}")
            if sol.fraction < 0.999:
                sys.exit("   El camino cartesiano no se completo (colision o IK): revisar escena/poses")
            tr = reparametrizar(sol.solution, perfil, p0, pf)
            m = metricas(tr, tr_def["v_max"], tr_def["a_max"])
            code, muestras = node.ejecutar(tr["t"], tr["q"], tr["qd"], tr["qdd"])
            te = np.array([s[0] for s in muestras]); Qe = np.array([s[1] for s in muestras])
            te = te - te[0]
            _, vte = tool_speed(te, Qe) if len(te) > 5 else (None, np.array([np.nan]))
            m.update({"ejecucion_codigo": code, "n_muestras_joint_states": len(te),
                      "v_tool_max_ejecutada": float(np.nanmax(vte)), "desvio_recta_mm": 1000 * tr["desvio_recta"]})
            print(f"   desvio de la recta = {m['desvio_recta_mm']:.3f} mm | qdd_max = {m['qdd_max']:.3f} rad/s^2 | "
                  f"salto qdd inicio/fin = {m['salto_qdd_inicio']:.3f}/{m['salto_qdd_final']:.3f} | jerk continuo max = {m['jerk_art_max_continuo']:.2f} | "
                  f"jerk del salto = {m['jerk_art_salto']:.1f} | int_jerk2 continuo = {m['int_jerk2_continuo']:.3f} | con saltos = {m['int_jerk2_con_saltos']:.3f}")
            print(f"   tool0: v_max plan = {m['v_tool_max']:.4f} m/s (lim {tr_def['v_max']}) cumple={m['cumple_v']} | "
                  f"a_max plan = {m['a_tool_max']:.4f} (lim {tr_def['a_max']}) cumple={m['cumple_a']} | "
                  f"v_max ejecutada = {m['v_tool_max_ejecutada']:.4f} | ejecucion codigo {code}")
            print(f"   referencia TOTG de MoveIt: T = {m['totg_T']:.2f} s, v_tool_max = {m['totg_v_tool_max']:.3f} m/s, qdd_max = {m['totg_qdd_max']:.2f}")
            if code != 1:
                sys.exit("   Fallo la ejecucion")
            q_actual = tr["q"][-1].tolist()
            salida.setdefault(tramo, {})[tipo] = {
                "metricas": m, "t": tr["t"].tolist(), "q": tr["q"].tolist(), "qd": tr["qd"].tolist(), "qdd": tr["qdd"].tolist(),
                "t_ejec": te.tolist(), "q_ejec": Qe.tolist(), "v_tool_ejec": vte.tolist(),
                "t_totg": tr["t_totg"].tolist(), "q_totg": tr["Q_totg"].tolist()}

    json.dump(salida, open(os.path.join(OUT, "4b_cartesiano.json"), "w"))

    graficas(salida, perfiles)
    print(f"\nGuardado en {OUT}: 4b_cartesiano.json, 4b_qdd_*.png, 4b_vtool_*.png")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
