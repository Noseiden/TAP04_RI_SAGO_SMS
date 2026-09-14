cmake_minimum_required(VERSION 3.22)
project(kr10_jacobiano_moveit)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(jacobiano_moveit src/jacobiano_moveit.cpp)
ament_target_dependencies(jacobiano_moveit rclcpp moveit_core moveit_ros_planning Eigen3)

install(TARGETS jacobiano_moveit DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
