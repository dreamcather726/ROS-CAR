"""
tf_tree_node

作用：
1. 订阅 /odom，把里程计位姿发布为动态 TF。
2. 发布底盘、IMU、雷达、摄像头之间的静态 TF。

运行：
ros2 run my_pkg tf_tree_node
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


DEFAULT_ODOM_TOPIC = "/odom"
DEFAULT_ODOM_FRAME = "odom"
DEFAULT_BASE_FRAME = "base_link"
DEFAULT_BASE_FOOTPRINT_FRAME = "base_footprint"
DEFAULT_IMU_FRAME = "imu_link"
DEFAULT_LASER_FRAME = "laser_link"
DEFAULT_CAMERA_FRAME = "camera_link"


def build_transform_message(
    parent_frame,
    child_frame,
    stamp,
    translation,
    rotation,
):
    """Build a TransformStamped message.

    Args:
        parent_frame: Parent TF frame name.
        child_frame: Child TF frame name.
        stamp: ROS2 time stamp.
        translation: Tuple of x, y, z in meters.
        rotation: Tuple of quaternion x, y, z, w.
    """
    transform_message = TransformStamped()
    transform_message.header.stamp = stamp
    transform_message.header.frame_id = parent_frame
    transform_message.child_frame_id = child_frame
    transform_message.transform.translation.x = float(translation[0])
    transform_message.transform.translation.y = float(translation[1])
    transform_message.transform.translation.z = float(translation[2])
    transform_message.transform.rotation.x = float(rotation[0])
    transform_message.transform.rotation.y = float(rotation[1])
    transform_message.transform.rotation.z = float(rotation[2])
    transform_message.transform.rotation.w = float(rotation[3])
    return transform_message


def build_quaternion_from_yaw(yaw_rad):
    """Build a quaternion for a yaw-only static transform."""
    half_yaw = yaw_rad * 0.5
    return (
        0.0,
        0.0,
        math.sin(half_yaw),
        math.cos(half_yaw),
    )


class TfTreeNode(Node):
    """Publish the robot TF tree."""

    def __init__(self):
        super().__init__("tf_tree_node")

        self.declare_parameter("odom_topic", DEFAULT_ODOM_TOPIC)
        self.declare_parameter("odom_frame", DEFAULT_ODOM_FRAME)
        self.declare_parameter("base_frame", DEFAULT_BASE_FRAME)
        self.declare_parameter(
            "base_footprint_frame",
            DEFAULT_BASE_FOOTPRINT_FRAME,
        )
        self.declare_parameter("imu_frame", DEFAULT_IMU_FRAME)
        self.declare_parameter("laser_frame", DEFAULT_LASER_FRAME)
        self.declare_parameter("camera_frame", DEFAULT_CAMERA_FRAME)

        self.declare_parameter("base_footprint_x", 0.0)
        self.declare_parameter("base_footprint_y", 0.0)
        self.declare_parameter("base_footprint_z", 0.0)
        self.declare_parameter("imu_x", 0.0)
        self.declare_parameter("imu_y", 0.0)
        self.declare_parameter("imu_z", 0.0)
        self.declare_parameter("laser_x", 0.0)
        self.declare_parameter("laser_y", 0.0)
        self.declare_parameter("laser_z", 0.0)
        self.declare_parameter("camera_x", 0.0)
        self.declare_parameter("camera_y", 0.0)
        self.declare_parameter("camera_z", 0.0)
        self.declare_parameter("camera_yaw", 0.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.base_footprint_frame = self.get_parameter(
            "base_footprint_frame",
        ).value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        odom_topic = self.get_parameter("odom_topic").value
        self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )

        self.publish_static_transforms()
        self.get_logger().info("tf_tree_node initialized successfully")

    def get_xyz_parameter(self, prefix):
        """Read x, y, z parameters for a static transform."""
        return (
            self.get_parameter(f"{prefix}_x").value,
            self.get_parameter(f"{prefix}_y").value,
            self.get_parameter(f"{prefix}_z").value,
        )

    def publish_static_transforms(self):
        """Publish static transforms from base_link to fixed child frames."""
        now_stamp = self.get_clock().now().to_msg()
        identity_rotation = (0.0, 0.0, 0.0, 1.0)
        camera_yaw = self.get_parameter("camera_yaw").value

        static_transforms = [
            build_transform_message(
                self.base_frame,
                self.base_footprint_frame,
                now_stamp,
                self.get_xyz_parameter("base_footprint"),
                identity_rotation,
            ),
            build_transform_message(
                self.base_frame,
                self.get_parameter("imu_frame").value,
                now_stamp,
                self.get_xyz_parameter("imu"),
                identity_rotation,
            ),
            build_transform_message(
                self.base_frame,
                self.get_parameter("laser_frame").value,
                now_stamp,
                self.get_xyz_parameter("laser"),
                identity_rotation,
            ),
            build_transform_message(
                self.base_frame,
                self.get_parameter("camera_frame").value,
                now_stamp,
                self.get_xyz_parameter("camera"),
                build_quaternion_from_yaw(camera_yaw),
            ),
        ]
        self.static_tf_broadcaster.sendTransform(static_transforms)

    def odom_callback(self, odom_message):
        """Publish odom to base_link dynamic transform."""
        parent_frame = odom_message.header.frame_id or self.odom_frame
        child_frame = odom_message.child_frame_id or self.base_frame
        pose = odom_message.pose.pose

        transform_message = build_transform_message(
            parent_frame,
            child_frame,
            odom_message.header.stamp,
            (
                pose.position.x,
                pose.position.y,
                pose.position.z,
            ),
            (
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ),
        )
        self.tf_broadcaster.sendTransform(transform_message)


def main(args=None):
    """Run tf_tree_node."""
    rclpy.init(args=args)
    node = TfTreeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
