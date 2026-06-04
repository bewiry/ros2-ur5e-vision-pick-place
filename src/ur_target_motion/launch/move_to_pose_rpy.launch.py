from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Launch arguments
    x_arg = DeclareLaunchArgument("x", default_value="0.30")
    y_arg = DeclareLaunchArgument("y", default_value="0.10")
    z_arg = DeclareLaunchArgument("z", default_value="0.40")

    roll_arg = DeclareLaunchArgument("roll", default_value="3.14")
    pitch_arg = DeclareLaunchArgument("pitch", default_value="0.0")
    yaw_arg = DeclareLaunchArgument("yaw", default_value="0.0")

    ur_type_arg = DeclareLaunchArgument("ur_type", default_value="ur5e")

    # Build MoveIt config from your existing package
    moveit_config = (
        MoveItConfigsBuilder("ur5e_workcell", package_name="ur_moveit_config")
        .to_moveit_configs()
    )

    move_to_pose_node = Node(
        package="ur_target_motion",
        executable="move_to_pose_rpy",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "x": LaunchConfiguration("x"),
                "y": LaunchConfiguration("y"),
                "z": LaunchConfiguration("z"),
                "roll": LaunchConfiguration("roll"),
                "pitch": LaunchConfiguration("pitch"),
                "yaw": LaunchConfiguration("yaw"),
            },
        ],
    )

    return LaunchDescription([
        ur_type_arg,
        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,
        move_to_pose_node,
    ])
