from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the YDLidar ROS2 driver node."""
    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("ydlidar_ros2_driver"),
        "params",
        "ydlidar.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="YDLidar ROS2 driver parameter file.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ydlidar",
            description="YDLidar serial port.",
        ),
        Node(
            package="ydlidar_ros2_driver",
            executable="ydlidar_ros2_driver_node",
            name="ydlidar_ros2_driver_node",
            output="screen",
            parameters=[
                params_file,
                {"port": port},
            ],
        ),
    ])
