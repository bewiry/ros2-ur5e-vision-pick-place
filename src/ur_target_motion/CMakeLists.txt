cmake_minimum_required(VERSION 3.8)
project(ur_target_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(move_to_point src/move_to_point.cpp)
ament_target_dependencies(move_to_point
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  moveit_msgs
)

add_executable(move_to_pose_rpy src/move_to_pose_rpy.cpp)
ament_target_dependencies(move_to_pose_rpy
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

add_executable(pick_place_node src/pick_place_node.cpp)
ament_target_dependencies(pick_place_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
  control_msgs
)

install(TARGETS
  move_to_point
  move_to_pose_rpy
  pick_place_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
