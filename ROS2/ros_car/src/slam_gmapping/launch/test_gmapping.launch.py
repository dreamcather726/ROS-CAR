from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start gmapping with the default package parameters."""
    params_file = LaunchConfiguration("params_file")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("slam_gmapping"),
        "params",
        "slam_gmapping.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="slam_gmapping parameter file.",
        ),
        Node(
            package="slam_gmapping",
            executable="slam_gmapping",
            output="screen",
            parameters=[params_file],
        ),
    ])
