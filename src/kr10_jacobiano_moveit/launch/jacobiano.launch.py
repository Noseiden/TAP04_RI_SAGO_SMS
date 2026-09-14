"""ros2 launch kr10_jacobiano_moveit jacobiano.launch.py entrada:=q_eval.csv salida:=J_moveit.csv"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    mc = MoveItConfigsBuilder("kr10_r1100_2", package_name="kr10_r1100_2_moveit_config").to_moveit_configs()
    return LaunchDescription([
        DeclareLaunchArgument("entrada"),
        DeclareLaunchArgument("salida"),
        Node(package="kr10_jacobiano_moveit", executable="jacobiano_moveit", output="screen",
             parameters=[mc.robot_description, mc.robot_description_semantic, mc.robot_description_kinematics,
                         {"entrada": LaunchConfiguration("entrada"), "salida": LaunchConfiguration("salida")}]),
    ])
