from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_ip = LaunchConfiguration("robot_ip")
    camera_index = LaunchConfiguration("camera_index")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_ip_arg = DeclareLaunchArgument(
        "robot_ip",
        default_value="192.168.1.102"
    )

    camera_index_arg = DeclareLaunchArgument(
        "camera_index",
        default_value="0"
    )

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true"
    )

    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false"
    )

    # Terminal 1: UR robot driver
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware
        }.items()
    )

    # Terminal 2: MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "launch",
                "ur_moveit.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": "ur5e",
            "launch_rviz": launch_rviz
        }.items()
    )

       # Terminal 3: Robotiq gripper URCap adapter
    robotiq_adapter_node = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'robotiq_2f_urcap_adapter',
            'robotiq_2f_adapter_node.py',
            '--ros-args',
            '-p',
            ['robot_ip:=', robot_ip]
        ],
        output='screen'
    )

    # Terminal 4: Vision node
    vision_node = Node(
        package="conveyor_cam",
        executable="tracker",
        name="box_tracker",
        output="screen",
        parameters=[
            {"camera_index": camera_index}
        ]
    )

    # Terminal 5: Final robot pick/place node
    inverse_node = Node(
        package="motion",
        executable="inverse",
        name="vision_moving_conveyor_pick_node",
        output="screen"
    )

    return LaunchDescription([
        robot_ip_arg,
        camera_index_arg,
        launch_rviz_arg,
        use_fake_hardware_arg,

        # Start robot driver immediately
        ur_driver_launch,

        # Start MoveIt after driver has time to come up
        TimerAction(
            period=8.0,
            actions=[moveit_launch]
        ),

        # Start gripper adapter
        TimerAction(
            period=12.0,
            actions=[robotiq_adapter_node]
        ),

        # Start camera tracking
        TimerAction(
            period=18.0,
            actions=[vision_node]
        ),

        # Start final pick/place logic last
        TimerAction(
            period=23.0,
            actions=[inverse_node]
        ),
    ])
