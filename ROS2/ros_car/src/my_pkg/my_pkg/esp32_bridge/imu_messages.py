"""IMU message builders for the ESP32 bridge."""

import math

from sensor_msgs.msg import Imu


def build_raw_imu_message(
    stamp,
    accel_x,
    accel_y,
    accel_z,
    gyro_x,
    gyro_y,
    gyro_z,
):
    """Build /imu/rawdata from MPU6050 raw acceleration and gyro data."""
    imu_msg = Imu()
    imu_msg.header.stamp = stamp
    imu_msg.header.frame_id = "imu_link"
    imu_msg.orientation_covariance[0] = -1.0
    imu_msg.linear_acceleration.x = float(accel_x)
    imu_msg.linear_acceleration.y = float(accel_y)
    imu_msg.linear_acceleration.z = float(accel_z)
    imu_msg.angular_velocity.x = float(gyro_x)
    imu_msg.angular_velocity.y = float(gyro_y)
    imu_msg.angular_velocity.z = float(gyro_z)
    return imu_msg


def build_resolved_imu_message(
    stamp,
    accel_x,
    accel_y,
    accel_z,
    gyro_x,
    gyro_y,
    gyro_z,
    roll_deg,
    pitch_deg,
    yaw_deg,
):
    """Build /imu/data with ESP32 resolved orientation.

    /imu/data 的 orientation 使用 ESP32 已解算姿态。
    linear_acceleration 和 angular_velocity 暂时仍使用 MPU6050 原始值。
    """
    roll_rad = math.radians(roll_deg)
    pitch_rad = math.radians(pitch_deg)
    yaw_rad = math.radians(yaw_deg)
    quaternion_x, quaternion_y, quaternion_z, quaternion_w = (
        build_quaternion_from_rpy(
            roll_rad,
            pitch_rad,
            yaw_rad,
        )
    )

    imu_msg = Imu()
    imu_msg.header.stamp = stamp
    imu_msg.header.frame_id = "imu_link"
    imu_msg.orientation.x = quaternion_x
    imu_msg.orientation.y = quaternion_y
    imu_msg.orientation.z = quaternion_z
    imu_msg.orientation.w = quaternion_w
    imu_msg.linear_acceleration.x = float(accel_x)
    imu_msg.linear_acceleration.y = float(accel_y)
    imu_msg.linear_acceleration.z = float(accel_z)
    imu_msg.angular_velocity.x = float(gyro_x)
    imu_msg.angular_velocity.y = float(gyro_y)
    imu_msg.angular_velocity.z = float(gyro_z)
    return imu_msg


def build_quaternion_from_rpy(roll, pitch, yaw):
    """Convert roll, pitch and yaw in radians to a quaternion."""
    half_roll = roll / 2.0
    half_pitch = pitch / 2.0
    half_yaw = yaw / 2.0

    sin_roll = math.sin(half_roll)
    cos_roll = math.cos(half_roll)
    sin_pitch = math.sin(half_pitch)
    cos_pitch = math.cos(half_pitch)
    sin_yaw = math.sin(half_yaw)
    cos_yaw = math.cos(half_yaw)

    quaternion_x = (
        sin_roll * cos_pitch * cos_yaw
        - cos_roll * sin_pitch * sin_yaw
    )
    quaternion_y = (
        cos_roll * sin_pitch * cos_yaw
        + sin_roll * cos_pitch * sin_yaw
    )
    quaternion_z = (
        cos_roll * cos_pitch * sin_yaw
        - sin_roll * sin_pitch * cos_yaw
    )
    quaternion_w = (
        cos_roll * cos_pitch * cos_yaw
        + sin_roll * sin_pitch * sin_yaw
    )
    return quaternion_x, quaternion_y, quaternion_z, quaternion_w
