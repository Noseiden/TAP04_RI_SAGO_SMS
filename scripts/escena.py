#!/usr/bin/env python3
"""Parte 4 - Escena de colision (PlanningScene) de la celda pick-and-place.

4 objetos en el marco base_link [m]:
  mesa_pick   caja 0.25 x 0.25 x 0.198, superficie en z = 0.198, bajo la pose pick
  pieza       cubo 0.04 entre z = 0.20 y 0.24 (tool0 en pick queda 1 cm arriba)
  poste       cilindro r = 0.05, alto 0.85, en (0.70, -0.10): corta el camino articular recto HOME -> pre-pick
  mesa_place  caja 0.25 x 0.25 x 0.298, superficie en z = 0.298, bajo la pose place
Las mesas quedan 2 mm por debajo de la pieza (en pick y en place): si la pieza "toca" la mesa, MoveIt lo cuenta
como colision cuando la pieza va pegada a la herramienta y el camino cartesiano se corta.

Uso:  python3 scripts/escena.py            (agrega / reemplaza los objetos)
      python3 scripts/escena.py --borrar   (quita los objetos)
"""
import sys

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive

FRAME = "base_link"
# nombre: (tipo, dimensiones, centro xyz)
OBJETOS = {
    "mesa_pick":  (SolidPrimitive.BOX,      [0.25, 0.25, 0.198], [0.55, -0.30, 0.099]),
    "pieza":      (SolidPrimitive.BOX,      [0.04, 0.04, 0.04], [0.55, -0.30, 0.22]),
    "poste":      (SolidPrimitive.CYLINDER, [0.85, 0.05],       [0.70, -0.10, 0.425]),  # [alto, radio]
    "mesa_place": (SolidPrimitive.BOX,      [0.25, 0.25, 0.298], [0.45, 0.40, 0.149]),
}


def objeto(nombre, tipo, dims, centro, operacion):
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = nombre
    co.operation = operacion
    if operacion == CollisionObject.ADD:
        co.primitives = [SolidPrimitive(type=tipo, dimensions=[float(d) for d in dims])]
        p = Pose()
        p.position.x, p.position.y, p.position.z = [float(c) for c in centro]
        p.orientation.w = 1.0
        co.primitive_poses = [p]
    return co


def main():
    borrar = "--borrar" in sys.argv
    rclpy.init()
    node = Node("parte4_escena")
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        sys.exit("No aparece /apply_planning_scene: ¿esta corriendo move_group?")

    scene = PlanningScene(is_diff=True)
    op = CollisionObject.REMOVE if borrar else CollisionObject.ADD
    scene.world.collision_objects = [objeto(n, *v, op) for n, v in OBJETOS.items()]
    fut = cli.call_async(ApplyPlanningScene.Request(scene=scene))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    ok = fut.result() is not None and fut.result().success
    print(("Objetos quitados: " if borrar else "Escena aplicada: ") + ", ".join(OBJETOS) if ok else "Fallo al aplicar la escena")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
