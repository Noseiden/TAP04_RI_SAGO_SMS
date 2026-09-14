"""Parte 3 - IK de las poses pick y place con el solver de MoveIt2 (KDL).

Requiere move_group corriendo (demo.launch.py de kr10_r1100_2_moveit_config).
Para cada pose:
  1. /compute_ik  -> angulos articulares. Semilla = HOME pero con A1 ya girado hacia la pieza:
                     KDL es numerico y converge a la solucion mas cercana a la semilla; desde HOME puro
                     puede caer en la rama de muneca volteada (q4 = +-180, q5 negativo).
  2. /compute_fk  -> pose de tool0 que MoveIt calcula con esos angulos (verifica la IK)
Guarda todo en resultados/ik_resultados.json, que lee matlab/parte3_ik_dh.m.

Uso (desde la raiz del repositorio):
  ros2 run kr10_taller ik_poses [--group manipulator] [--tip tool0] [--base base_link]
"""
import argparse
import json
import math
import os
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINTS = [f"joint_{i}" for i in range(1, 7)]  # variante 2 (kuka_agilus_support)
HOME = [0.0, -math.pi / 2, math.pi / 2, 0.0, 0.0, 0.0]



def tool_down(p):
    """Herramienta vertical hacia abajo y alineada con la direccion radial base -> pieza.

    R = Rz(psi) * Ry(180 deg), psi = atan2(y, x)  ->  cuaternion [x y z w] = [-sin(psi/2), cos(psi/2), 0, 0].
    Es la postura de HOME girada con A1 y con la muneca doblada 90 deg: evita que A4/A6 tengan que compensar.
    """
    psi = math.atan2(p[1], p[0])
    return {"p": p, "q": [-math.sin(psi / 2), math.cos(psi / 2), 0.0, 0.0]}


# Poses objetivo de tool0 en base_link [m]
POSES = {
    "pick":  tool_down([0.550, -0.300, 0.250]),
    "place": tool_down([0.450, 0.400, 0.350]),
}


class IKClient(Node):
    def __init__(self, group, tip, base):
        super().__init__("parte3_ik_client")
        self.group, self.tip, self.base = group, tip, base
        self.ik = self.create_client(GetPositionIK, "/compute_ik")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        for cli, name in ((self.ik, "/compute_ik"), (self.fk, "/compute_fk")):
            if not cli.wait_for_service(timeout_sec=15.0):
                sys.exit(f"No aparece {name}: ¿esta corriendo move_group?")

    def call(self, cli, req):
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        if fut.result() is None:
            sys.exit("El servicio no respondio")
        return fut.result()

    def solve_ik(self, p, q):
        req = GetPositionIK.Request()
        r = req.ik_request
        r.group_name = self.group
        r.ik_link_name = self.tip
        r.avoid_collisions = False
        r.timeout.sec = 1
        seed = list(HOME)
        seed[0] = -math.atan2(p[1], p[0])  # signo menos: A1 gira alrededor de -z en el URDF de KUKA
        r.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=seed))
        r.pose_stamped = PoseStamped()
        r.pose_stamped.header.frame_id = self.base
        pose = r.pose_stamped.pose
        pose.position.x, pose.position.y, pose.position.z = p
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
        res = self.call(self.ik, req)
        if res.error_code.val != 1:
            return None
        js = res.solution.joint_state
        return [js.position[js.name.index(j)] for j in JOINTS]

    def solve_fk(self, qj):
        req = GetPositionFK.Request()
        req.header.frame_id = self.base
        req.fk_link_names = [self.tip]
        req.robot_state = RobotState(joint_state=JointState(name=JOINTS, position=qj))
        res = self.call(self.fk, req)
        ps = res.pose_stamped[0].pose
        return ([ps.position.x, ps.position.y, ps.position.z],
                [ps.orientation.x, ps.orientation.y, ps.orientation.z, ps.orientation.w])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="manipulator")
    ap.add_argument("--tip", default="tool0")
    ap.add_argument("--base", default="base_link")
    ap.add_argument("--out", default=os.path.join("resultados", "ik_resultados.json"),
                    help="ruta del JSON de salida (relativa a la carpeta desde donde se ejecuta)")
    args = ap.parse_args()

    rclpy.init()
    node = IKClient(args.group, args.tip, args.base)
    out = {"group": args.group, "tip": args.tip, "base": args.base, "joints": JOINTS, "poses": {}}
    for name, target in POSES.items():
        qj = node.solve_ik(target["p"], target["q"])
        if qj is None:
            sys.exit(f"MoveIt no encontro IK para '{name}': ajuste la pose (fuera de alcance o de limites)")
        p_fk, q_fk = node.solve_fk(qj)
        out["poses"][name] = {"target_p": target["p"], "target_q_xyzw": target["q"],
                              "q_rad": qj, "fk_p": p_fk, "fk_q_xyzw": q_fk}
        print(f"\n== {name} ==")
        print("  objetivo  p =", target["p"], " q(xyzw) =", [round(v, 4) for v in target["q"]])
        print("  IK [deg]    =", [round(math.degrees(v), 3) for v in qj])
        print("  FK MoveIt p =", [round(v, 4) for v in p_fk], " q(xyzw) =", [round(v, 4) for v in q_fk])

    path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print(f"\nGuardado en {path}")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
