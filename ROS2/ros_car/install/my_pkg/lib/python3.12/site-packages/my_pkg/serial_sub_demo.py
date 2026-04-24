import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Imu, Range


class SerialSubDemo(Node):
    def __init__(self):
        super().__init__('serial_sub_demo')

        self.declare_parameter('topic_ns', '/car')
        topic_ns = str(self.get_parameter('topic_ns').value).rstrip('/')

        self.create_subscription(Vector3Stamped, f'{topic_ns}/odom/wheel_speed_mps', self.on_wheel_speed, 10)
        self.create_subscription(TwistStamped, f'{topic_ns}/twist', self.on_twist, 10)
        self.create_subscription(Range, f'{topic_ns}/ultrasonic/range', self.on_range, 10)
        self.create_subscription(Imu, f'{topic_ns}/imu/data', self.on_imu, 10)
        self.create_subscription(DiagnosticArray, f'{topic_ns}/serial/status', self.on_status, 10)

        self.get_logger().info(f'✅ serial_sub_demo 已启动 | topic_ns={topic_ns}')

    def on_wheel_speed(self, msg: Vector3Stamped):
        self.get_logger().info(f'wheel_speed_mps | left={msg.vector.x:.3f} right={msg.vector.y:.3f}')

    def on_twist(self, msg: TwistStamped):
        self.get_logger().info(f'twist | v={msg.twist.linear.x:.3f} m/s w={msg.twist.angular.z:.3f} rad/s')

    def on_range(self, msg: Range):
        self.get_logger().info(f'range | {msg.range:.3f} m')

    def on_imu(self, msg: Imu):
        self.get_logger().info(
            f'imu | acc=({msg.linear_acceleration.x:.3f},{msg.linear_acceleration.y:.3f},{msg.linear_acceleration.z:.3f}) '
            f'm/s^2 gyro=({msg.angular_velocity.x:.3f},{msg.angular_velocity.y:.3f},{msg.angular_velocity.z:.3f}) rad/s'
        )

    def on_status(self, msg: DiagnosticArray):
        if not msg.status:
            return
        st = msg.status[0]
        kv = {kv.key: kv.value for kv in st.values}
        self.get_logger().info(
            f'serial_status | {st.message} port={kv.get("port","")} ok={kv.get("ok_frames","0")} '
            f'bad_tail={kv.get("bad_tail","0")} checksum_fail={kv.get("checksum_fail","0")} unknown={kv.get("unknown_func","0")}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SerialSubDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
