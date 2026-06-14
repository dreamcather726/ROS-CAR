from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch all robot nodes in my_pkg."""
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    wheel_base_m = LaunchConfiguration("wheel_base_m")
    enable_print = LaunchConfiguration("enable_print")
    odom_publish_rate_hz = LaunchConfiguration("odom_publish_rate_hz")
    use_imu_yaw_for_odom = LaunchConfiguration("use_imu_yaw_for_odom")
    use_keyboard_control = LaunchConfiguration("use_keyboard_control")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    ydlidar_driver_launch = LaunchConfiguration("ydlidar_driver_launch")
    tf_params_file = PathJoinSubstitution([
        FindPackageShare("my_pkg"),
        "config",
        "tf_params.yaml",
    ])
    ydlidar_launch_file = PathJoinSubstitution([
        FindPackageShare("ydlidar_ros2_driver"),
        "launch",
        ydlidar_driver_launch,
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyUSB1",
            description="Serial port connected to ESP32.",
        ),
        DeclareLaunchArgument(
            "baudrate",
            default_value="115200",
            description="ESP32 serial baudrate.",
        ),
        DeclareLaunchArgument(
            "wheel_base_m",
            default_value="0.18",
            description="Distance between left and right drive wheels.",
        ),
        DeclareLaunchArgument(
            "enable_print",
            default_value="false",
            description="Enable normal debug logs from esp32_bridge_node.",
        ),
        DeclareLaunchArgument(
            "odom_publish_rate_hz",
            default_value="20.0",
            description="Maximum /odom publish rate. Use 0 for no limit.",
        ),
        DeclareLaunchArgument(
            "use_imu_yaw_for_odom",
            default_value="true",
            description="Use ESP32 resolved IMU yaw to correct /odom yaw.",
        ),
        DeclareLaunchArgument(
            "use_keyboard_control",
            default_value="false",
            description="Start keyboard_control_node for W/A/S/D teleop.",
        ),
        DeclareLaunchArgument(
            "use_lidar",
            default_value="false",
            description="Start ydlidar_ros2_driver and publish /scan.",
        ),
        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/ydlidar",
            description="Serial port connected to YDLidar.",
        ),
        DeclareLaunchArgument(
            "ydlidar_driver_launch",
            default_value="ydlidar_launch.py",
            description="Launch file from ydlidar_ros2_driver.",
        ),
        Node(
            package="my_pkg",
            executable="esp32_bridge_node",
            name="esp32_bridge_node",
            output="screen",
            parameters=[{
                "port": port,
                "baudrate": baudrate,
                "wheel_base_m": wheel_base_m,
                "enable_print": enable_print,
                "odom_publish_rate_hz": odom_publish_rate_hz,
                "use_imu_yaw_for_odom": use_imu_yaw_for_odom,
            }],
        ),
        Node(
            package="my_pkg",
            executable="tf_tree_node",
            name="tf_tree_node",
            output="screen",
            parameters=[tf_params_file],
        ),
        Node(
            package="my_pkg",
            executable="keyboard_control_node",
            name="keyboard_control_node",
            output="screen",
            condition=IfCondition(use_keyboard_control),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ydlidar_launch_file),
            condition=IfCondition(use_lidar),
            launch_arguments={
                "port": lidar_port,
            }.items(),
        ),
    ])
