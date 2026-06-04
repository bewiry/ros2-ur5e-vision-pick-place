cmake_minimum_required(VERSION 3.8)
project(motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Threads REQUIRED)
find_package(robotiq_2f_urcap_adapter REQUIRED)

# =========================
# Executable 1: inverse
# Final vision + moving conveyor + gripper node
# =========================
add_executable(inverse src/inverse.cpp)

ament_target_dependencies(inverse
  rclcpp
  rclcpp_action
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
  robotiq_2f_urcap_adapter
)

target_link_libraries(inverse
  Threads::Threads
)

target_compile_features(inverse PUBLIC cxx_std_17)

# =========================
# Executable 2: cartesian_pick_place
# =========================
add_executable(cartesian_pick_place src/cartesian_pick_place.cpp)

ament_target_dependencies(cartesian_pick_place
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

target_link_libraries(cartesian_pick_place
  Threads::Threads
)

target_compile_features(cartesian_pick_place PUBLIC cxx_std_17)

# =========================
# Executable 3: conveyor_speed_estimator
# =========================
add_executable(conveyor_speed_estimator src/conveyor_speed_estimator.cpp)

ament_target_dependencies(conveyor_speed_estimator
  rclcpp
  geometry_msgs
)

target_compile_features(conveyor_speed_estimator PUBLIC cxx_std_17)

# =========================
# Install executables
# =========================
install(TARGETS
  inverse
  cartesian_pick_place
  conveyor_speed_estimator
  DESTINATION lib/${PROJECT_NAME}
)

# =========================
# Install launch files
# =========================
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
