# TAP02_RI_SMS_SGO – Pick-and-place con KUKA KR10 R1100-2 en ROS2 + MoveIt2

Universidad EIA · Robótica Industrial · Taller ROS2/MoveIt2 (Parcial 2) · Autores: SMS, SGO

## Contenido

| Carpeta | Qué hay |
|---|---|
| `src/kr10_r1100_2_description` | URDF/xacro y mallas del robot |
| `src/kr10_r1100_2_moveit_config` | Configuración de MoveIt2 hecha con el Setup Assistant (parte 1) |
| `src/kr10_jacobiano_moveit` | Nodo C++ con `RobotState::getJacobian()` (parte 5) |
| `scripts/` | Python: IK, escena, planeadores, perfiles, ciclo, configuraciones del Jacobiano |
| `matlab/` | DH en HOME (parte 2), IK vs DH (parte 3), Jacobiano analítico (parte 5) |

Todos los resultados se escriben en `resultados/` (se crea al ejecutar).

## Requisitos

Ubuntu 24.04, ROS 2 Jazzy, MoveIt 2 (`sudo apt install ros-jazzy-moveit`), `python3-scipy`, `python3-matplotlib`,
MATLAB con Symbolic Math Toolbox y Robotics System Toolbox.

## Compilar

```bash
git clone git@github.com:Noseiden/Parcial2_Robotica.git TAP02_RI_SMS_SGO
cd TAP02_RI_SMS_SGO
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

Todo lo siguiente se ejecuta desde la raíz del repositorio con `install/setup.bash` cargado.

## Ejecutar

**Terminal 1 (dejarla abierta en todas las partes):**

```bash
ros2 launch kr10_r1100_2_moveit_config demo.launch.py     # arranca en HOME
```

### Parte 2 – Transformación homogénea en HOME

```bash
mkdir -p resultados
timeout 5 ros2 run tf2_ros tf2_echo base_link tool0 > resultados/tf_home.txt
```

MATLAB: `run matlab/parte2_dh_home.m`

### Parte 3 – IK de pick y place

```bash
python3 scripts/ik_poses.py
```

MATLAB: `run matlab/parte3_ik_dh.m`

### Parte 4 – Escena, planeadores, perfiles y ciclo

```bash
python3 scripts/escena.py                   # mesa_pick, pieza, poste, mesa_place
python3 scripts/comparar_planeadores_4a.py  # 4A: RRTConnect vs RRTstar (15 intentos, ~3 min)
python3 scripts/perfiles_4b.py              # 4B: perfiles cúbico y quíntico
python3 scripts/cartesiano_4b.py            # 4B: computeCartesianPath + re-parametrización, ejecuta ambos perfiles
python3 scripts/ciclo_4.py                  # ciclo completo 4A→4B→4C→4D (RRTConnect + quíntico)
```

### Parte 5 – Jacobiano

```bash
python3 scripts/configuraciones_5.py
ros2 launch kr10_jacobiano_moveit jacobiano.launch.py entrada:=$PWD/resultados/q_eval.csv salida:=$PWD/resultados/J_moveit.csv
```

MATLAB: `run matlab/parte5_jacobiano.m`

## Créditos

URDF y mallas de [kroshu/kuka_robot_descriptions](https://github.com/kroshu/kuka_robot_descriptions)
(`kuka_agilus_support`, Apache-2.0, ver `src/kr10_r1100_2_description/LICENSE`).
