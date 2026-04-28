import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistStamped, Vector3Stamped
from sensor_msgs.msg import Range

import serial
import serial.tools.list_ports

class SerialSensorNode(Node):
    def __init__(self, *, start_serial: Optional[bool] = None):
        super().__init__('serial_node')

        self.declare_parameter('port', '/dev/ttyUSB0')# 串口端口
        self.declare_parameter('baudrate', 115200)# 串口波特率
        self.declare_parameter('topic_ns', '/car')# 主题命名空间
        self.declare_parameter('wheel_base_m', 0.0)# 轮距
        self.declare_parameter('rx_timeout_s', 2.0)# 接收超时时间
        self.declare_parameter('reconnect_interval_s', 1.0)# 重新连接间隔
        self.declare_parameter('warn_interval_s', 2.0)# 警告间隔
        self.declare_parameter('enable_serial', True)   # 是否启用串口通信

        self.port = str(self.get_parameter('port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.topic_ns = str(self.get_parameter('topic_ns').value).rstrip('/')
        self.wheel_base_m = float(self.get_parameter('wheel_base_m').value)
        self.rx_timeout_s = float(self.get_parameter('rx_timeout_s').value)
        self.reconnect_interval_s = float(self.get_parameter('reconnect_interval_s').value)
        self.warn_interval_s = float(self.get_parameter('warn_interval_s').value)
        self.enable_serial = bool(self.get_parameter('enable_serial').value)
        if start_serial is not None:
            self.enable_serial = bool(start_serial)

        self._frame_len = 10
        self._start_byte = 0xAA
        self._end_byte = 0xBB
        
        self._FUNC_ODOM_COUNTS = 0x01
        self._FUNC_ODOM_SPEED = 0x02
        self._FUNC_ODOM_DISTANCE = 0x03
        self._FUNC_IMU_RPY = 0x04

        self._frame_id_base = 'base_link'
        self._frame_id_imu = 'imu_link'
        self._frame_id_range = 'range_link'

        self.pub_odom_counts = self.create_publisher(Vector3Stamped, f'{self.topic_ns}/odom/counts', 10)
        self.pub_wheel_speed = self.create_publisher(Vector3Stamped, f'{self.topic_ns}/odom/wheel_speed_mps', 10)
        self.pub_twist = self.create_publisher(TwistStamped, f'{self.topic_ns}/twist', 10)
        self.pub_range = self.create_publisher(Range, f'{self.topic_ns}/ultrasonic/range', 10)
        self.pub_rpy = self.create_publisher(Vector3Stamped, f'{self.topic_ns}/imu/rpy_deg', 10)
        self.pub_status = self.create_publisher(DiagnosticArray, f'{self.topic_ns}/serial/status', 10)

        self._ser = None
        self._rx_buf = bytearray()
        self._stop_evt = threading.Event()
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)

        self._last_rx_monotonic = 0.0
        self._last_reconnect_try_monotonic = 0.0
        self._last_warn_monotonic = 0.0
        self._bad_tail_count = 0
        self._checksum_fail_count = 0
        self._unknown_func_count = 0
        self._frames_ok = 0
        self._connected_port = None

        self._last_rpy_deg = None

        if self.enable_serial:
            self._reader_thread.start()

        self.create_timer(0.2, self._watchdog_tick)
        self.get_logger().info(
            f"✅ serial_node 已启动 | port={self.port} baud={self.baudrate} ns={self.topic_ns}"
        )

    def _pick_port(self) -> str:
        if self.port and self.port.lower() != 'auto':
            return self.port

        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if p.device:
                return p.device
        return self.port

    def open_serial(self):
        try:
            port = self._pick_port()
            self._ser = serial.Serial(
                port=port,
                baudrate=self.baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.05
            )
            self._rx_buf.clear()
            self._last_rx_monotonic = time.monotonic()
            self._connected_port = port
            self.get_logger().info(f"✅ 串口打开成功：{port}")
        except Exception as e:
            self._ser = None
            self._connected_port = None
            self._warn_throttled(f"❌ 串口打开失败：{str(e)}")

    def _close_serial(self):
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self._ser = None
        self._connected_port = None

    def _warn_throttled(self, msg: str):
        now = time.monotonic()
        if now - self._last_warn_monotonic >= self.warn_interval_s:
            self._last_warn_monotonic = now
            self.get_logger().warn(msg)

    @staticmethod
    def _read_i16_le(data: bytes, offset: int) -> int:
        v = data[offset] | (data[offset + 1] << 8)
        if v & 0x8000:
            v -= 0x10000
        return int(v)

    @staticmethod
    def _read_i24_le(data: bytes, offset: int) -> int:
        v = data[offset] | (data[offset + 1] << 8) | (data[offset + 2] << 16)
        if v & 0x800000:
            v -= 0x1000000
        return int(v)

    def inject_bytes(self, data: bytes):
        self._parse_bytes(data)

    def _on_frame(self, func: int, payload: bytes):
        now = self.get_clock().now().to_msg()

        if func == self._FUNC_ODOM_COUNTS:
            left = self._read_i24_le(payload, 0)
            right = self._read_i24_le(payload, 3)
            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id_base
            msg.vector.x = float(left)
            msg.vector.y = float(right)
            msg.vector.z = 0.0
            self.pub_odom_counts.publish(msg)
            return

        if func == self._FUNC_ODOM_SPEED:
            left_raw = self._read_i16_le(payload, 0)
            right_raw = self._read_i16_le(payload, 2)
            left_mps = (left_raw / 100.0) / 100.0
            right_mps = (right_raw / 100.0) / 100.0

            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id_base
            msg.vector.x = float(left_mps)
            msg.vector.y = float(right_mps)
            msg.vector.z = 0.0
            self.pub_wheel_speed.publish(msg)

            twist = TwistStamped()
            twist.header.stamp = now
            twist.header.frame_id = self._frame_id_base
            twist.twist.linear.x = float((left_mps + right_mps) * 0.5)
            twist.twist.angular.z = (
                float((right_mps - left_mps) / self.wheel_base_m)
                if self.wheel_base_m and self.wheel_base_m > 0.0
                else 0.0
            )
            self.pub_twist.publish(twist)
            return

        if func == self._FUNC_ODOM_DISTANCE:
            dist_raw = self._read_i16_le(payload, 0)
            dist_m = float('nan') if dist_raw == -1 else float((dist_raw / 10.0) / 100.0)
            msg = Range()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id_range
            msg.radiation_type = Range.ULTRASOUND
            msg.field_of_view = 0.26
            msg.min_range = 0.02
            msg.max_range = 4.0
            msg.range = dist_m
            self.pub_range.publish(msg)
            return

        if func == self._FUNC_IMU_RPY:
            roll_deg = self._read_i16_le(payload, 0) / 100.0
            pitch_deg = self._read_i16_le(payload, 2) / 100.0
            yaw_deg = self._read_i16_le(payload, 4) / 100.0
            self._last_rpy_deg = (roll_deg, pitch_deg, yaw_deg)
            msg = Vector3Stamped()
            msg.header.stamp = now
            msg.header.frame_id = self._frame_id_imu
            msg.vector.x = float(roll_deg)
            msg.vector.y = float(pitch_deg)
            msg.vector.z = float(yaw_deg)
            self.pub_rpy.publish(msg)
            return

        self._unknown_func_count += 1

    def _parse_bytes(self, data: bytes):
        self._rx_buf.extend(data)

        while True:
            start = self._rx_buf.find(bytes([self._start_byte]))
            if start < 0:
                if len(self._rx_buf) > self._frame_len:
                    del self._rx_buf[:-self._frame_len]
                return

            if start > 0:
                del self._rx_buf[:start]

            if len(self._rx_buf) < self._frame_len:
                return

            frame = bytes(self._rx_buf[:self._frame_len])
            if frame[-1] != self._end_byte:
                self._bad_tail_count += 1
                del self._rx_buf[0:1]
                continue

            calc_sum = sum(frame[0:8]) & 0xFF
            recv_sum = frame[8]
            if calc_sum != recv_sum:
                self._checksum_fail_count += 1
                del self._rx_buf[0:1]
                continue

            del self._rx_buf[:self._frame_len]
            self._frames_ok += 1
            self._on_frame(frame[1], frame[2:8])

    def _reader_loop(self):
        while not self._stop_evt.is_set():
            if not self._ser or not self._ser.is_open:
                now = time.monotonic()
                if now - self._last_reconnect_try_monotonic >= self.reconnect_interval_s:
                    self._last_reconnect_try_monotonic = now
                    self.open_serial()
                time.sleep(0.02)
                continue

            try:
                n = self._ser.in_waiting
                chunk = self._ser.read(n if n else 1)
            except Exception as e:
                self._warn_throttled(f"⚠️ 串口读取异常，准备重连：{str(e)}")
                self._close_serial()
                time.sleep(self.reconnect_interval_s)
                continue

            if chunk:
                self._last_rx_monotonic = time.monotonic()
                self._parse_bytes(chunk)
            else:
                time.sleep(0.002)

    def _watchdog_tick(self):
        now = time.monotonic()
        if self._ser and self._ser.is_open and (now - self._last_rx_monotonic) > self.rx_timeout_s:
            self._warn_throttled(f"⏱️ 串口接收超时({self.rx_timeout_s:.1f}s)，准备重连")
            self._close_serial()

        diag = DiagnosticArray()
        diag.header.stamp = self.get_clock().now().to_msg()
        st = DiagnosticStatus()
        st.name = f'{self.topic_ns}/serial'
        st.hardware_id = 'esp32s3'
        st.level = DiagnosticStatus.OK if (self._ser and self._ser.is_open) else DiagnosticStatus.WARN
        st.message = 'connected' if (self._ser and self._ser.is_open) else 'disconnected'
        st.values = [
            KeyValue(key='port', value=str(self._connected_port) if self._connected_port else ''),
            KeyValue(key='ok_frames', value=str(self._frames_ok)),
            KeyValue(key='bad_tail', value=str(self._bad_tail_count)),
            KeyValue(key='checksum_fail', value=str(self._checksum_fail_count)),
            KeyValue(key='unknown_func', value=str(self._unknown_func_count)),
        ]
        diag.status.append(st)
        self.pub_status.publish(diag)

        if (self._bad_tail_count + self._checksum_fail_count + self._unknown_func_count) > 0:
            self._warn_throttled(
                f"帧异常统计 | ok={self._frames_ok} bad_tail={self._bad_tail_count} "
                f"checksum_fail={self._checksum_fail_count} unknown_func={self._unknown_func_count}"
            )

    def destroy_node(self):
        self._stop_evt.set()
        self._close_serial()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SerialSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
