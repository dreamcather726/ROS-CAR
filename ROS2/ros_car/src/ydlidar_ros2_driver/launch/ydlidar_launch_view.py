from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the YDLidar driver and RViz."""
    params_file = LaunchConfiguration("params_file")
    port = LaunchConfiguration("port")
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("ydlidar_ros2_driver"),
        "rviz",
        "ydlidar.rviz",
    ])
    driver_launch_file = PathJoinSubstitution([
        FindPackageShare("ydlidar_ros2_driver"),
        "launch",
        "ydlidar_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("ydlidar_ros2_driver"),
                "params",
                "ydlidar.yaml",
            ]),
            description="YDLidar ROS2 driver parameter file.",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ydlidar",
            description="YDLidar serial port.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(driver_launch_file),
            launch_arguments={
                "params_file": params_file,
                "port": port,
            }.items(),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config_file],
            output="screen",
        ),
    ])
