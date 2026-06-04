from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction


def terminal_command(title, command):
    return ExecuteProcess(
        cmd=[
            "gnome-terminal",
            "--title", title,
            "--",
            "bash",
            "-lc",
            command + "; echo ''; echo 'Terminal process ended. Press Enter to keep/close.'; read"
        ],
        output="screen"
    )


def generate_launch_description():

    terminal_1_ur_driver = (
        "cd ~/ur_driver && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch ur_robot_driver ur_control.launch.py "
        "ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=false"
    )

    terminal_2_moveit = (
        "cd ~/ur_driver && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 launch ur_moveit_config ur_moveit.launch.py "
        "ur_type:=ur5e launch_rviz:=true"
    )

    terminal_3_gripper_calibration = (
        "cd ~/ur_driver && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run robotiq_2f_urcap_adapter robotiq_2f_adapter_node.py "
        "--ros-args -p robot_ip:=192.168.1.102"
    )

    terminal_4_vision = (
        "cd ~/ur_driver && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run conveyor_cam tracker --ros-args -p camera_index:=2"
    )

    terminal_5_inverse = (
        "cd ~/ur_driver && "
        "source /opt/ros/humble/setup.bash && "
        "source install/setup.bash && "
        "ros2 run motion inverse"
    )

    return LaunchDescription([

        terminal_command(
            "1 - UR Driver",
            terminal_1_ur_driver
        ),

        TimerAction(
            period=8.0,
            actions=[
                terminal_command(
                    "2 - MoveIt",
                    terminal_2_moveit
                )
            ]
        ),

        TimerAction(
            period=12.0,
            actions=[
                terminal_command(
                    "3 - Gripper Calibration / Robotiq Adapter",
                    terminal_3_gripper_calibration
                )
            ]
        ),

        TimerAction(
            period=20.0,
            actions=[
                terminal_command(
                    "4 - Vision Node",
                    terminal_4_vision
                )
            ]
        ),

        TimerAction(
            period=26.0,
            actions=[
                terminal_command(
                    "5 - Final Motion Inverse Node",
                    terminal_5_inverse
                )
            ]
        ),
    ])
