from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start the time-decayed scan overlay map."""
    decay_seconds = LaunchConfiguration("decay_seconds")
    width_m = LaunchConfiguration("width_m")
    height_m = LaunchConfiguration("height_m")
    origin_x = LaunchConfiguration("origin_x")
    origin_y = LaunchConfiguration("origin_y")
    hit_increment = LaunchConfiguration("hit_increment")

    return LaunchDescription([
        DeclareLaunchArgument(
            "decay_seconds",
            default_value="8.0",
            description="Seconds before scan-hit cells fade out.",
        ),
        DeclareLaunchArgument(
            "width_m",
            default_value="20.0",
            description="Overlay map width in meters.",
        ),
        DeclareLaunchArgument(
            "height_m",
            default_value="20.0",
            description="Overlay map height in meters.",
        ),
        DeclareLaunchArgument(
            "origin_x",
            default_value="-10.0",
            description="Overlay map origin x in the map frame.",
        ),
        DeclareLaunchArgument(
            "origin_y",
            default_value="-10.0",
            description="Overlay map origin y in the map frame.",
        ),
        DeclareLaunchArgument(
            "hit_increment",
            default_value="25.0",
            description="Score added to each hit cell.",
        ),
        Node(
            package="my_pkg",
            executable="scan_decay_map_node",
            name="scan_decay_map_node",
            output="screen",
            parameters=[{
                "decay_seconds": ParameterValue(
                    decay_seconds,
                    value_type=float,
                ),
                "width_m": ParameterValue(width_m, value_type=float),
                "height_m": ParameterValue(height_m, value_type=float),
                "origin_x": ParameterValue(origin_x, value_type=float),
                "origin_y": ParameterValue(origin_y, value_type=float),
                "hit_increment": ParameterValue(
                    hit_increment,
                    value_type=float,
                ),
            }],
        ),
    ])
