"""Muestra el KR10 R1100-2 en RViz2.

  ros2 launch kr10_r1100_2_description display.launch.py            # robot fijo en HOME
  ros2 launch kr10_r1100_2_description display.launch.py gui:=true # deslizadores para mover articulaciones

Con gui:=false se usa joint_state_publisher sin GUI a proposito: los deslizadores
redondean los angulos (q2 queda en -89.97 deg) y la comparacion con DH deja de ser exacta.
"""
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# HOME [deg] = [0, -90, 90, 0, 0, 0]
HOME = {"joint_1": 0.0, "joint_2": -math.pi / 2, "joint_3": math.pi / 2,
        "joint_4": 0.0, "joint_5": 0.0, "joint_6": 0.0}


def generate_launch_description():
    pkg = FindPackageShare("kr10_r1100_2_description")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([pkg, "urdf", "kr10_r1100_2.urdf.xacro"])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description}]),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             parameters=[{"zeros": HOME}], condition=UnlessCondition(gui)),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             parameters=[{"zeros": HOME}], condition=IfCondition(gui)),
        Node(package="rviz2", executable="rviz2", condition=IfCondition(rviz),
             arguments=["-d", PathJoinSubstitution([pkg, "rviz", "display.rviz"])]),
    ])
