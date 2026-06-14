"""
esp32_bridge_node

作用：
1. 订阅 /cmd_vel，把小车线速度和角速度转换成左右轮速度。
2. 通过串口把左右轮目标速度发送给 ESP32。
3. 接收 ESP32 编码器和 IMU 数据。
4. 发布 /odom、/imu/rawdata、/imu/data 和 /esp32/status。

运行：
ros2 run my_pkg esp32_bridge_node --ros-args -p port:=/dev/ttyUSB0
"""

import math
import struct
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

from my_pkg.esp32_bridge.imu_messages import (
    build_raw_imu_message,
    build_resolved_imu_message,
)
from my_pkg.esp32_bridge.odometry import (
    WheelOdometry,
    limit_wheel_speed,
)
from my_pkg.esp32_bridge.serial_protocol import (
    FUNC_ENCODER,
    FUNC_IMU,
    FUNC_WHEEL_SPEED,
    build_serial_frame,
    read_int16_le,
    read_int24_le,
    receive_serial_messages,
    serial,
)


DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUDRATE = 115200
WHEEL_BASE_M = 0.18
RECONNECT_INTERVAL_SEC = 5.0
PRINT_INTERVAL_SEC = 1.0
DEFAULT_ENABLE_PRINT = False
DEFAULT_ODOM_PUBLISH_RATE_HZ = 20.0
DEFAULT_USE_IMU_YAW_FOR_ODOM = True
MAX_WHEEL_SPEED_CM_S = 50.0


class Esp32BridgeNode(Node):
    """Bridge ROS2 topics and the ESP32 serial protocol."""

    def __init__(self):
        super().__init__("esp32_bridge_node")

        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE)
        self.declare_parameter("wheel_base_m", WHEEL_BASE_M)
        self.declare_parameter("enable_print", DEFAULT_ENABLE_PRINT)
        self.declare_parameter(
            "odom_publish_rate_hz",
            DEFAULT_ODOM_PUBLISH_RATE_HZ,
        )
        self.declare_parameter(
            "use_imu_yaw_for_odom",
            DEFAULT_USE_IMU_YAW_FOR_ODOM,
        )

        wheel_base_m = self.get_parameter("wheel_base_m").value
        self.wheel_odometry = WheelOdometry(wheel_base_m)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.imu_raw_pub = self.create_publisher(Imu, "/imu/rawdata", 10)
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.status_pub = self.create_publisher(String, "/esp32/status", 10)

        self.serial_port = None
        self.receive_buffer = bytearray()
        self.last_encoder_print_time = 0.0
        self.last_imu_print_time = 0.0
        self.last_other_print_time = 0.0
        self.last_cmd_vel_print_time = 0.0
        self.last_odom_publish_time = 0.0
        self.latest_imu_yaw = None

        self.open_serial_port()

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10)
        self.create_timer(0.02, self.timer_callback)
        self.create_timer(
            RECONNECT_INTERVAL_SEC,
            self.reconnect_timer_callback,
        )
        self.publish_status("initialized")
        self.get_logger().info("esp32_bridge_node initialized successfully")

    def publish_status(self, text):
        """Publish ESP32 connection status."""
        status_msg = String()
        status_msg.data = text
        self.status_pub.publish(status_msg)

    def open_serial_port(self):
        """Open the configured serial port."""
        if serial is None:
            self.get_logger().error("pyserial is not installed")
            return

        if self.serial_port is not None and self.serial_port.is_open:
            return

        port = self.get_parameter("port").value
        baudrate = self.get_parameter("baudrate").value

        try:
            self.serial_port = serial.Serial(
                port,
                baudrate,
                timeout=0.005,
                write_timeout=0.2,
            )
        except serial.SerialException as error:
            self.serial_port = None
            self.get_logger().error(f"failed to open {port}: {error}")
            return

        self.get_logger().info(f"opened serial port {port} at {baudrate}")
        self.publish_status("serial_connected")

    def close_serial_port(self):
        """Close serial port after read/write failure."""
        if self.serial_port is None:
            return

        try:
            self.serial_port.close()
        except serial.SerialException as error:
            self.get_logger().warning(f"failed to close serial port: {error}")

        self.serial_port = None
        self.receive_buffer.clear()
        self.get_logger().warning("serial port closed, wait reconnect")
        self.publish_status("serial_disconnected")

    def reconnect_timer_callback(self):
        """Try to reopen serial port after disconnect."""
        if serial is None:
            return

        if self.serial_port is None:
            self.open_serial_port()

    def can_print_now(self, last_print_time):
        """Return whether normal debug output can be printed now."""
        enable_print = self.get_parameter("enable_print").value
        if not enable_print:
            return False, last_print_time

        now = time.monotonic()
        if now - last_print_time < PRINT_INTERVAL_SEC:
            return False, last_print_time
        return True, now

    def timer_callback(self):
        """Read serial data and dispatch parsed ESP32 messages."""
        if self.serial_port is None or not self.serial_port.is_open:
            return

        messages = receive_serial_messages(
            self.serial_port,
            self.receive_buffer,
            self.get_logger(),
        )

        if messages is None:
            self.close_serial_port()
            return

        for func, payload in messages:
            self.handle_serial_message(func, payload)

    def handle_serial_message(self, func, payload):
        """Handle one parsed ESP32 protocol message."""
        if func == FUNC_ENCODER:
            self.handle_encoder_message(payload)
            return

        if func == FUNC_IMU:
            self.handle_imu_message(payload)
            return

        can_print, self.last_other_print_time = self.can_print_now(
            self.last_other_print_time
        )
        if not can_print:
            return

        payload_hex = " ".join(f"{one_byte:02X}" for one_byte in payload)
        self.get_logger().info(
            f"received func=0x{func:02X} payload={payload_hex}"
        )

    def handle_encoder_message(self, payload):
        """Parse encoder data and publish odometry."""
        if len(payload) < 6:
            self.get_logger().warning("encoder payload is too short")
            return

        left_count = read_int24_le(payload, 0)
        right_count = read_int24_le(payload, 3)

        if len(payload) >= 10:
            left_speed = read_int16_le(payload, 6) / 100.0
            right_speed = read_int16_le(payload, 8) / 100.0
            self.publish_odom(left_speed, right_speed)

        can_print, self.last_encoder_print_time = self.can_print_now(
            self.last_encoder_print_time
        )
        if not can_print:
            return

        if len(payload) >= 10:
            self.get_logger().info(
                "encoder: "
                f"left_count={left_count}, "
                f"right_count={right_count}, "
                f"left_speed={left_speed:.2f}cm/s, "
                f"right_speed={right_speed:.2f}cm/s"
            )
            return

        self.get_logger().info(
            f"encoder: left_count={left_count}, right_count={right_count}"
        )

    def publish_odom(self, left_speed_cm_s, right_speed_cm_s):
        """Update odometry from wheel speeds and publish at a limited rate."""
        odom_msg = self.wheel_odometry.build_message(
            left_speed_cm_s,
            right_speed_cm_s,
            self.get_clock().now(),
            self.get_odom_imu_yaw(),
        )

        odom_publish_rate_hz = float(
            self.get_parameter("odom_publish_rate_hz").value
        )
        if odom_publish_rate_hz > 0.0:
            now = time.monotonic()
            publish_interval_sec = 1.0 / odom_publish_rate_hz
            if now - self.last_odom_publish_time < publish_interval_sec:
                return
            self.last_odom_publish_time = now

        self.odom_pub.publish(odom_msg)

    def handle_imu_message(self, payload):
        """Parse IMU data and publish raw and resolved IMU messages."""
        if len(payload) < 18:
            self.get_logger().warning("imu payload is too short")
            return

        accel_x = read_int16_le(payload, 0)
        accel_y = read_int16_le(payload, 2)
        accel_z = read_int16_le(payload, 4)
        gyro_x = read_int16_le(payload, 6)
        gyro_y = read_int16_le(payload, 8)
        gyro_z = read_int16_le(payload, 10)
        roll = read_int16_le(payload, 12) / 10.0
        pitch = read_int16_le(payload, 14) / 10.0
        yaw = read_int16_le(payload, 16) / 10.0
        self.latest_imu_yaw = math.radians(yaw)

        stamp = self.get_clock().now().to_msg()
        self.imu_raw_pub.publish(
            build_raw_imu_message(
                stamp,
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
            )
        )
        self.imu_pub.publish(
            build_resolved_imu_message(
                stamp,
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
                roll,
                pitch,
                yaw,
            )
        )

        can_print, self.last_imu_print_time = self.can_print_now(
            self.last_imu_print_time
        )
        if not can_print:
            return

        self.get_logger().info(
            "imu: "
            f"accel=({accel_x},{accel_y},{accel_z}), "
            f"gyro=({gyro_x},{gyro_y},{gyro_z}), "
            f"rpy=({roll:.1f},{pitch:.1f},{yaw:.1f})deg"
        )

    def get_odom_imu_yaw(self):
        """Return latest IMU yaw when odometry fusion is enabled."""
        use_imu_yaw_for_odom = self.get_parameter(
            "use_imu_yaw_for_odom"
        ).value
        if not use_imu_yaw_for_odom:
            return None
        return self.latest_imu_yaw

    def cmd_vel_callback(self, msg):
        """Convert /cmd_vel into ESP32 left and right wheel speed frame."""
        wheel_base_m = self.get_parameter("wheel_base_m").value
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        left_speed_m_s = linear_speed - angular_speed * wheel_base_m / 2.0
        right_speed_m_s = linear_speed + angular_speed * wheel_base_m / 2.0
        left_speed_cm_s = limit_wheel_speed(
            left_speed_m_s * 100.0,
            MAX_WHEEL_SPEED_CM_S,
        )
        right_speed_cm_s = limit_wheel_speed(
            right_speed_m_s * 100.0,
            MAX_WHEEL_SPEED_CM_S,
        )

        left_raw = int(round(left_speed_cm_s * 100.0))
        right_raw = int(round(right_speed_cm_s * 100.0))
        payload = struct.pack("<hh", left_raw, right_raw)
        frame = build_serial_frame(FUNC_WHEEL_SPEED, payload)

        if self.serial_port is None:
            self.get_logger().warning("serial port is not open")
            return

        try:
            self.serial_port.write(frame)
        except serial.SerialException as error:
            self.get_logger().error(f"failed to send cmd_vel: {error}")
            self.close_serial_port()
            return

        can_print, self.last_cmd_vel_print_time = self.can_print_now(
            self.last_cmd_vel_print_time
        )
        if not can_print:
            return

        self.get_logger().info(
            "send cmd_vel: "
            f"left={left_speed_cm_s:.2f}cm/s, "
            f"right={right_speed_cm_s:.2f}cm/s"
        )

    def destroy_node(self):
        if self.serial_port is not None:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Esp32BridgeNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
