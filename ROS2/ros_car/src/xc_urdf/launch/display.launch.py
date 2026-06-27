from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Publish the XC car URDF for RViz2 visualization."""
    use_rviz = LaunchConfiguration("use_rviz")
    package_share = Path(get_package_share_directory("xc_urdf"))
    urdf_file = package_share / "urdf" / "xc_urdf.urdf"
    robot_description = urdf_file.read_text(encoding="utf-8")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Start RViz2 together with robot_state_publisher.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": False,
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(use_rviz),
        ),
    ])
