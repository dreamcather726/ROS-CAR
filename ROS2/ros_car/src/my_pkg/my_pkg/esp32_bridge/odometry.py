"""Odometry helpers for the ESP32 bridge."""

import math

from nav_msgs.msg import Odometry


class WheelOdometry:
    """Integrate wheel speeds into an Odometry message."""

    def __init__(self, wheel_base_m):
        self.wheel_base_m = wheel_base_m
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.last_time = None

    def build_message(self, left_speed_cm_s, right_speed_cm_s, now):
        """Build /odom from left and right wheel speed.

        参数：
            left_speed_cm_s：左驱动轮实际速度，单位 cm/s。
            right_speed_cm_s：右驱动轮实际速度，单位 cm/s。
            now：ROS2 当前时间。
        """
        if self.last_time is None:
            time_delta_s = 0.0
        else:
            time_delta_s = (
                now - self.last_time
            ).nanoseconds / 1000000000.0

        self.last_time = now

        left_speed_m_s = left_speed_cm_s / 100.0
        right_speed_m_s = right_speed_cm_s / 100.0
        linear_speed = (left_speed_m_s + right_speed_m_s) / 2.0
        angular_speed = (
            right_speed_m_s - left_speed_m_s
        ) / self.wheel_base_m

        if time_delta_s > 0.0:
            self.position_x += (
                linear_speed * math.cos(self.yaw) * time_delta_s
            )
            self.position_y += (
                linear_speed * math.sin(self.yaw) * time_delta_s
            )
            self.yaw += angular_speed * time_delta_s

        quaternion_x, quaternion_y, quaternion_z, quaternion_w = (
            build_quaternion_from_yaw(self.yaw)
        )

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.pose.pose.position.y = self.position_y
        odom_msg.pose.pose.orientation.x = quaternion_x
        odom_msg.pose.pose.orientation.y = quaternion_y
        odom_msg.pose.pose.orientation.z = quaternion_z
        odom_msg.pose.pose.orientation.w = quaternion_w
        odom_msg.twist.twist.linear.x = linear_speed
        odom_msg.twist.twist.angular.z = angular_speed
        return odom_msg


def build_quaternion_from_yaw(yaw):
    """Build an odometry quaternion from yaw in radians."""
    half_yaw = yaw / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def limit_wheel_speed(speed_cm_s, max_wheel_speed_cm_s):
    """Limit target wheel speed before sending it to ESP32."""
    if speed_cm_s > max_wheel_speed_cm_s:
        return max_wheel_speed_cm_s
    if speed_cm_s < -max_wheel_speed_cm_s:
        return -max_wheel_speed_cm_s
    return speed_cm_s
