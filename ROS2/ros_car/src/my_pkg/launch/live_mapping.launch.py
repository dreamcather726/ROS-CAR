from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start only GMapping for live map publishing."""
    slam_params_file = LaunchConfiguration("slam_params_file")

    default_slam_params_file = PathJoinSubstitution([
        FindPackageShare("slam_gmapping"),
        "params",
        "slam_gmapping.yaml",
    ])
    slam_gmapping_launch_file = PathJoinSubstitution([
        FindPackageShare("slam_gmapping"),
        "launch",
        "gmapping_x3_launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "slam_params_file",
            default_value=default_slam_params_file,
            description="GMapping parameter file.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_gmapping_launch_file),
            launch_arguments={
                "params_file": slam_params_file,
            }.items(),
        ),
    ])
