import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Range


class SerialSubDemo(Node):
    def __init__(self):
        super().__init__('serial_sub_demo')

        self.declare_parameter('topic_ns', '/car')
        topic_ns = str(self.get_parameter('topic_ns').value).rstrip('/')

        self.create_subscription(Vector3Stamped, f'{topic_ns}/odom/wheel_speed_mps', self.on_wheel_speed, 10)
        self.create_subscription(TwistStamped, f'{topic_ns}/twist', self.on_twist, 10)
        self.create_subscription(Range, f'{topic_ns}/ultrasonic/range', self.on_range, 10)
        self.create_subscription(Vector3Stamped, f'{topic_ns}/imu/rpy_deg', self.on_rpy, 10)
        self.create_subscription(DiagnosticArray, f'{topic_ns}/serial/status', self.on_status, 10)

        self._wheel_speed = None
        self._twist = None
        self._range = None
        self._rpy = None
        self._status = None

        self.create_timer(1.0, self._print_tick)

        self.get_logger().info(f'✅ serial_sub_demo 已启动 | topic_ns={topic_ns}')

    def on_wheel_speed(self, msg: Vector3Stamped):
        self._wheel_speed = msg

    def on_twist(self, msg: TwistStamped):
        self._twist = msg

    def on_range(self, msg: Range):
        self._range = msg

    def on_rpy(self, msg: Vector3Stamped):
        self._rpy = msg

    def on_status(self, msg: DiagnosticArray):
        self._status = msg

    def _print_tick(self):
        parts = []

        if self._wheel_speed:
            parts.append(
                f'wheel_speed_mps left={self._wheel_speed.vector.x:.3f} right={self._wheel_speed.vector.y:.3f}'
            )

        if self._twist:
            parts.append(
                f'twist v={self._twist.twist.linear.x:.3f} w={self._twist.twist.angular.z:.3f}'
            )

        if self._range:
            parts.append(f'range {self._range.range:.3f}m')

        if self._rpy:
            parts.append(
                f'rpy_deg roll={self._rpy.vector.x:.2f} pitch={self._rpy.vector.y:.2f} yaw={self._rpy.vector.z:.2f}'
            )

        if self._status and self._status.status:
            st = self._status.status[0]
            kv = {kv.key: kv.value for kv in st.values}
            parts.append(
                f'serial {st.message} port={kv.get("port","")} ok={kv.get("ok_frames","0")} '
                f'bad_tail={kv.get("bad_tail","0")} checksum_fail={kv.get("checksum_fail","0")} unknown={kv.get("unknown_func","0")}'
            )

        if parts:
            self.get_logger().info(' | '.join(parts))


def main(args=None):
    rclpy.init(args=args)
    node = SerialSubDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
