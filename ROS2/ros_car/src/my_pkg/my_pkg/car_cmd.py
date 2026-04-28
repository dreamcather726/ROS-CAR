import rclpy
import struct
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8MultiArray

# 串口协议
HEADER = 0xAA
TAIL = 0xBB
FUNC_WHEEL_SPEED = 0x10

def _checksum8(b: bytes) -> int:
    return sum(b) & 0xFF

def _build_frame(func: int, payload6: bytes) -> bytes:
    if len(payload6) != 6:
        raise ValueError("payload must be 6 bytes")
    head = bytes([HEADER, func & 0xFF]) + payload6
    chk = _checksum8(head)
    return head + bytes([chk, TAIL])

def build_wheel_speed_frame(left_cm_s: float, right_cm_s: float) -> bytes:
    l = int(round(left_cm_s * 100.0))
    r = int(round(right_cm_s * 100.0))
    l = max(-32768, min(32767, l))
    r = max(-32768, min(32767, r))
    payload = struct.pack("<hh", l, r) + b"\x00\x00"
    return _build_frame(FUNC_WHEEL_SPEED, payload)

class CarCmdNode(Node):
    def __init__(self):
        super().__init__('car_cmd_node')
        self.declare_parameter('wheel_track', 0.18)
        self.wheel_track = self.get_parameter('wheel_track').value

        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)
        self.pub_tx = self.create_publisher(UInt8MultiArray, '/serial/tx_frame', 10)

        self.left = 0.0
        self.right = 0.0
        self.create_timer(0.05, self.send_timer)

    def cb(self, msg):
        vx = msg.linear.x
        vw = msg.angular.z
        left_mps  = vx - vw * self.wheel_track / 2.0
        right_mps = vx + vw * self.wheel_track / 2.0
        self.left  = left_mps * 100
        self.right = right_mps * 100

    def send_timer(self):
        frame = build_wheel_speed_frame(self.left, self.right)
        arr = UInt8MultiArray()
        arr.data = list(frame)
        self.pub_tx.publish(arr)

def main(args=None):
    rclpy.init(args=args)
    node = CarCmdNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()