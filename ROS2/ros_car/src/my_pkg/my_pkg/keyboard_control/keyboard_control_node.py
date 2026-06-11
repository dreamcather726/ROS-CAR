"""
keyboard_control_node

作用：
1. 读取键盘 W/A/S/D。
2. 发布 /cmd_vel，控制小车前进、后退、左转、右转。
3. 键盘短时间没有输入时自动发布停止速度。

运行：
ros2 run my_pkg keyboard_control_node

按键：
W: 前进
S: 后退
A: 左转
D: 右转
Space/X: 停止
Ctrl-C: 退出
"""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


DEFAULT_LINEAR_SPEED_M_S = 0.10
DEFAULT_ANGULAR_SPEED_RAD_S = 0.80
DEFAULT_PUBLISH_RATE_HZ = 10.0
DEFAULT_KEY_TIMEOUT_SEC = 0.30
CMD_VEL_TOPIC = "/cmd_vel"


class KeyboardReader:
    """Read one keyboard key without waiting for Enter."""

    def __init__(self):
        self.original_terminal_settings = None
        self.is_windows = sys.platform.startswith("win")

    def __enter__(self):
        if self.is_windows:
            return self

        import termios
        import tty

        self.original_terminal_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self.is_windows or self.original_terminal_settings is None:
            return

        import termios

        termios.tcsetattr(
            sys.stdin,
            termios.TCSADRAIN,
            self.original_terminal_settings,
        )

    def read_key(self):
        """Return one key if available, otherwise return None."""
        if self.is_windows:
            import msvcrt

            if not msvcrt.kbhit():
                return None
            return msvcrt.getwch().lower()

        import select

        ready_inputs, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready_inputs:
            return None

        return sys.stdin.read(1).lower()


class KeyboardControlNode(Node):
    """Publish /cmd_vel from W/A/S/D keyboard input."""

    def __init__(self, keyboard_reader):
        super().__init__("keyboard_control_node")

        self.declare_parameter("linear_speed_m_s", DEFAULT_LINEAR_SPEED_M_S)
        self.declare_parameter(
            "angular_speed_rad_s",
            DEFAULT_ANGULAR_SPEED_RAD_S,
        )
        self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ)
        self.declare_parameter("key_timeout_sec", DEFAULT_KEY_TIMEOUT_SEC)

        publish_rate_hz = self.get_parameter("publish_rate_hz").value
        timer_period_sec = 1.0 / float(publish_rate_hz)

        self.keyboard_reader = keyboard_reader
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        self.last_key_time = 0.0
        self.create_timer(timer_period_sec, self.timer_callback)

        self.get_logger().info(
            "keyboard_control_node initialized: "
            "W/S forward/backward, A/D turn, Space/X stop"
        )

    def timer_callback(self):
        """Read keyboard and publish the latest command velocity."""
        key = self.keyboard_reader.read_key()
        if key is not None:
            self.handle_key(key)

        self.stop_when_key_timeout()
        self.publish_cmd_vel()

    def handle_key(self, key):
        """Convert one keyboard key to target linear and angular speed."""
        linear_speed_m_s = self.get_parameter("linear_speed_m_s").value
        angular_speed_rad_s = self.get_parameter(
            "angular_speed_rad_s",
        ).value

        if key == "w":
            self.current_linear_speed = linear_speed_m_s
            self.current_angular_speed = 0.0
        elif key == "s":
            self.current_linear_speed = -linear_speed_m_s
            self.current_angular_speed = 0.0
        elif key == "a":
            self.current_linear_speed = 0.0
            self.current_angular_speed = angular_speed_rad_s
        elif key == "d":
            self.current_linear_speed = 0.0
            self.current_angular_speed = -angular_speed_rad_s
        elif key == " " or key == "x":
            self.current_linear_speed = 0.0
            self.current_angular_speed = 0.0
        else:
            return

        self.last_key_time = time.monotonic()

    def stop_when_key_timeout(self):
        """Stop robot if no valid key has been received recently."""
        key_timeout_sec = self.get_parameter("key_timeout_sec").value
        if time.monotonic() - self.last_key_time <= key_timeout_sec:
            return

        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0

    def publish_cmd_vel(self):
        """Publish current command velocity to /cmd_vel."""
        twist_msg = Twist()
        twist_msg.linear.x = float(self.current_linear_speed)
        twist_msg.angular.z = float(self.current_angular_speed)
        self.cmd_vel_pub.publish(twist_msg)


def main(args=None):
    """Run keyboard_control_node."""
    rclpy.init(args=args)

    with KeyboardReader() as keyboard_reader:
        node = KeyboardControlNode(keyboard_reader)
        try:
            rclpy.spin(node)
        finally:
            stop_msg = Twist()
            node.cmd_vel_pub.publish(stop_msg)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
