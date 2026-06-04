# ROS 2 UR5e Vision-Based Pick and Place System

This project implements an industrial robotic pick-and-place system using a UR5e robot, ROS 2, MoveIt, an overhead camera, ArUco-based conveyor calibration, and a Robotiq 2F gripper.

The system detects colored boxes moving on a conveyor, estimates the conveyor speed, predicts when the box reaches a fixed picking line, transforms the conveyor coordinates into the robot base frame, picks the box, and places it on the table based on its color.

## Features

- UR5e robot motion control using ROS 2 and MoveIt
- Cartesian robot movement for approach, pick, lift, place, and retreat
- Robotiq gripper control using ROS 2 actions
- Overhead camera detection of colored boxes
- ArUco marker calibration for conveyor coordinate frame
- Conveyor-to-robot coordinate transformation
- Conveyor speed estimation
- Predictive picking from a moving conveyor
- Color-based sorting for green, red, and blue boxes
- Launch file for running the full system

## System Pipeline

Camera detection  
→ Conveyor-frame object position  
→ Conveyor speed estimation  
→ Prediction at fixed picking line  
→ Transformation to robot base frame  
→ UR5e Cartesian motion  
→ Robotiq gripper pick  
→ Color-based placement

## Main Packages

- `motion`: robot motion, moving conveyor prediction, pick-and-place logic, color sorting
- `conveyor_cam`: camera-based box detection and conveyor coordinate publishing
- `robotiq_2f_urcap_adapter`: Robotiq gripper adapter
- UR robot driver and MoveIt configuration packages

## Build

```bash
cd ~/ur_driver
colcon build
source install/setup.bash



## Run full system 

ros2 launch motion full_ir_project.launch.py

## Run the system manually

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true
ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py --ros-args -p robot_ip:=192.168.1.102
ros2 run conveyor_cam tracker --ros-args -p camera_index:=1
ros2 run motion inverse






