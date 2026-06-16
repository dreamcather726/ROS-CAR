from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start downsampled-scan gmapping and RViz for desktop map viewing."""
    params_file = LaunchConfiguration("params_file")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    default_params_file = PathJoinSubstitution([
        FindPackageShare("slam_gmapping"),
        "params",
        "slam_gmapping.yaml",
    ])
    default_rviz_config_file = PathJoinSubstitution([
        FindPackageShare("slam_gmapping"),
        "rviz",
        "view_gmapping.rviz",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="slam_gmapping parameter file.",
        ),
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=default_rviz_config_file,
            description="RViz config file for mapping.",
        ),
        Node(
            package="slam_gmapping",
            executable="slam_gmapping",
            output="screen",
            parameters=[params_file],
            remappings=[("/scan", "/downsampled_scan")],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        ),
    ])
