from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start Nav2 with a saved map for the ROS2 car."""
    map_file = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")

    default_map_file = PathJoinSubstitution([
        EnvironmentVariable("HOME"),
        "ros2_car",
        "maps",
        "first_map.yaml",
    ])
    default_params_file = PathJoinSubstitution([
        FindPackageShare("my_pkg"),
        "config",
        "nav2_params.yaml",
    ])
    nav2_bringup_launch_file = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "bringup_launch.py",
    ])
    nav2_rviz_launch_file = PathJoinSubstitution([
        FindPackageShare("nav2_bringup"),
        "launch",
        "rviz_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "map",
            default_value=default_map_file,
            description="Saved map yaml file for Nav2 localization.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Nav2 parameter file.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation clock.",
        ),
        DeclareLaunchArgument(
            "autostart",
            default_value="true",
            description="Automatically start Nav2 lifecycle nodes.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start Nav2 RViz on this machine.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_bringup_launch_file),
            launch_arguments={
                "map": map_file,
                "use_sim_time": use_sim_time,
                "params_file": params_file,
                "autostart": autostart,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_rviz_launch_file),
            condition=IfCondition(use_rviz),
            launch_arguments={
                "use_sim_time": use_sim_time,
            }.items(),
        ),
    ])
