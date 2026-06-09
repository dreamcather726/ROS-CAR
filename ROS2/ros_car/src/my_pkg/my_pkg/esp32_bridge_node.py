"""
esp32_bridge_node

程序逻辑：
1. 打开 Muse Pi 和 ESP32 之间的串口。
2. 循环接收 ESP32 发来的串口数据帧。
3. 解析编码器和 IMU 数据，并发布成 ROS2 话题。
4. 接收 ROS2 的 /cmd_vel 速度指令，转换成左右轮速度后发送给 ESP32。

订阅：
- /cmd_vel  geometry_msgs/msg/Twist

发布：
- /odom  nav_msgs/msg/Odometry
- /imu/data  sensor_msgs/msg/Imu
- /esp32/status  std_msgs/msg/String

打印：
- 默认关闭正常数据打印。
- 需要调试时启动参数加：-p enable_print:=true
- 运行中也可以打开：ros2 param set /esp32_bridge_node enable_print true

串口协议：
- 帧格式：AA LEN FUNC PAYLOAD CRC_L CRC_H BB
- 0x01：编码器数据
- 0x02：IMU 数据
- 0x10：发送给 ESP32 的左右轮目标速度
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

try:
    import serial
except ImportError:
    serial = None


FRAME_HEADER = 0xAA
FRAME_TAIL = 0xBB
MAX_PAYLOAD_SIZE = 32

FUNC_ENCODER = 0x01
FUNC_IMU = 0x02
FUNC_WHEEL_SPEED = 0x10

DEFAULT_PORT = "/dev/ttyUSB0"  # 串口设备路径，也可以改成 COM7 等。
DEFAULT_BAUDRATE = 115200
WHEEL_BASE_M = 0.18  # 左右驱动轮中心距，单位米，图纸尺寸是 180 mm。
RECONNECT_INTERVAL_SEC = 1.0 # 串口断开后的重连间隔，单位秒，避免频繁重试。
PRINT_INTERVAL_SEC = 1.0 # 普通调试日志的最小打印间隔，单位秒，避免日志过多。
DEFAULT_ENABLE_PRINT = False # 是否启用正常数据的打印，默认关闭，避免日志过多。调试时可以打开。
MAX_WHEEL_SPEED_CM_S = 50.0 # 最大轮速，单位 cm/s，限制在安全范围内，避免下位机过载。


# 从串口读取数据，解析成完整数据帧。
def receive_serial_messages(serial_port, receive_buffer, logger):
    """从串口读取数据，解析成完整数据帧。

    参数：
        serial_port：已经打开的 pyserial 串口对象。
        receive_buffer：串口接收缓冲区，保存还没处理完的字节。
        logger：ROS2 日志对象，用来打印错误和警告。

    返回：
        list：解析成功的消息列表，每个元素是 (func, payload)。
        None：串口读取失败，需要上层关闭串口并等待重连。
    """
    try:
        received_bytes = serial_port.read(128)
    except serial.SerialException as error:
        logger.error(f"failed to read serial data: {error}")
        return None

    if not received_bytes:
        return []

    receive_buffer.extend(received_bytes)
    messages = []

    # 最小帧长度是 6 字节：帧头 + 长度 + 功能码 + CRC + 帧尾。
    while len(receive_buffer) >= 6:
        frame_start = receive_buffer.find(bytes([FRAME_HEADER]))
        if frame_start < 0:
            receive_buffer.clear()
            break

        if frame_start > 0:
            del receive_buffer[:frame_start]

        if len(receive_buffer) < 6:
            break

        data_length = receive_buffer[1]
        frame_length = 1 + 1 + data_length + 2 + 1

        if data_length < 1 or data_length > MAX_PAYLOAD_SIZE + 1:
            del receive_buffer[0]
            continue

        if len(receive_buffer) < frame_length:
            break

        frame = bytes(receive_buffer[:frame_length])
        del receive_buffer[:frame_length]

        message = parse_serial_frame(frame, logger)
        if message is not None:
            messages.append(message)

    if len(receive_buffer) > 256:
        logger.warning("serial receive buffer is too large, clear it")
        receive_buffer.clear()

    return messages


# 解析串口数据帧，检查帧头、帧尾和 CRC，返回功能码和 payload。
def parse_serial_frame(frame, logger):
    """检查帧头、帧尾和 CRC。

    参数：
        frame：一帧完整的串口数据，格式是 AA LEN FUNC PAYLOAD CRC BB。
        logger：ROS2 日志对象，用来打印校验失败原因。

    返回：
        tuple：校验成功时返回 (func, payload)。
        None：校验失败时返回 None。
    """
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        logger.warning("dropped serial frame: bad header or tail")
        return None

    data_length = frame[1]
    frame_data = frame[2:2 + data_length]
    received_crc = frame[2 + data_length] | (frame[3 + data_length] << 8)
    calculated_crc = calculate_crc16(bytes([data_length]) + frame_data)

    if received_crc != calculated_crc:
        logger.warning(
            "dropped serial frame: "
            f"bad crc recv=0x{received_crc:04X} "
            f"calc=0x{calculated_crc:04X}"
        )
        return None

    func = frame_data[0]
    payload = frame_data[1:]
    return func, payload


# CRC16-Modbus，和 ESP32 下位机保持一致。
def calculate_crc16(data_bytes):
    """计算 CRC16-Modbus 校验值。

    参数：
        data_bytes：参与 CRC 计算的字节数据。

    返回：
        int：16 位 CRC 校验值。
    """
    crc = 0xFFFF
    for one_byte in data_bytes:
        crc ^= one_byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


# 从 payload 中读取小端 int16。
def read_int16_le(payload, offset):
    """从 payload 中读取一个小端 int16。

    参数：
        payload：串口帧中的业务数据。
        offset：从 payload 的第几个字节开始读取。

    返回：
        int：解析后的有符号 16 位整数。
    """
    return struct.unpack_from("<h", payload, offset)[0]


# 从 payload 中读取小端 int24，注意符号扩展。
def read_int24_le(payload, offset):
    """从 payload 中读取一个小端 int24。

    参数：
        payload：串口帧中的业务数据。
        offset：从 payload 的第几个字节开始读取。

    返回：
        int：解析后的有符号 24 位整数，适合编码器计数。
    """
    value = (
        payload[offset]
        | (payload[offset + 1] << 8)
        | (payload[offset + 2] << 16)
    )
    if value & 0x800000:
        value -= 0x1000000
    return value


# 构建发送给 ESP32 的串口数据帧，包含功能码和 payload。
def build_serial_frame(func, payload):
    """构建发送给 ESP32 的串口帧。

    参数：
        func：功能码，例如 0x10 表示左右轮目标速度。
        payload：功能码后面的业务数据。

    返回：
        bytes：完整串口帧，格式是 AA LEN FUNC PAYLOAD CRC BB。
    """
    frame_data = bytes([func]) + payload
    data_length = len(frame_data)
    crc = calculate_crc16(bytes([data_length]) + frame_data)

    return (
        bytes([FRAME_HEADER, data_length])
        + frame_data
        + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FRAME_TAIL])
    )


# 限制轮速在最大值范围内，避免下位机过载。
def limit_wheel_speed(speed_cm_s):
    """限制左右轮目标速度。

    参数：
        speed_cm_s：轮子目标速度，单位 cm/s。

    返回：
        float：限制在安全范围内的速度。
    """
    if speed_cm_s > MAX_WHEEL_SPEED_CM_S:
        return MAX_WHEEL_SPEED_CM_S
    if speed_cm_s < -MAX_WHEEL_SPEED_CM_S:
        return -MAX_WHEEL_SPEED_CM_S
    return speed_cm_s


def build_quaternion_from_yaw(yaw):
    """根据 yaw 角生成 odom 使用的四元数。

    参数：
        yaw：小车在 odom 坐标系里的朝向，单位弧度。

    返回：
        tuple：四元数 (x, y, z, w)。
    """
    half_yaw = yaw / 2.0
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


# ESP32 桥接节点，负责串口通信和 ROS2 交互。
class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__("esp32_bridge_node")

        self.declare_parameter("port", DEFAULT_PORT) # 串口设备路径参数，默认是 /dev/ttyUSB0，Windows 用户可以改成 COM7 等。
        self.declare_parameter("baudrate", DEFAULT_BAUDRATE) # 串口波特率参数，默认是 115200，根据实际情况修改。
        self.declare_parameter("wheel_base_m", WHEEL_BASE_M)# 左右轮中心距参数，单位米，图纸尺寸是 180 mm，根据实际情况修改。
        self.declare_parameter("enable_print", DEFAULT_ENABLE_PRINT) # 是否启用正常数据打印参数，默认是 false，避免日志过多，调试时可以打开。

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10) # 发布里程计话题，消息类型是 nav_msgs/msg/Odometry，队列长度 10。
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10) # 发布 IMU 数据话题，消息类型是 sensor_msgs/msg/Imu，队列长度 10。
        self.status_pub = self.create_publisher(String, "/esp32/status", 10) # 发布 ESP32 连接状态话题，消息类型是 std_msgs/msg/String，队列长度 10。

        self.serial_port = None # pyserial 串口对象，初始化为 None，表示还没有打开串口。
        self.receive_buffer = bytearray() #
        self.wheel_base_m = self.get_parameter("wheel_base_m").value #
        # 里程计当前位置（机器人在世界坐标系的坐标）
        self.odom_x = 0.0                    # 机器人 X 坐标
        self.odom_y = 0.0                    # 机器人 Y 坐标
        self.odom_yaw = 0.0                  # 机器人朝向角（偏航角，就是车头朝哪）
        self.last_odom_time = None           # 上一次计算里程计的时间（用来算时间差 dt）
        # 下面这些都是【日志打印限速】用的，防止每秒打印几百次刷屏
        self.last_encoder_print_time = 0.0   #上一次打印编码器数据的时间
        self.last_imu_print_time = 0.0       # 上一次打印IMU数据的时间
        self.last_other_print_time = 0.0     # 上一次打印其他数据的时间
        self.last_cmd_vel_print_time = 0.0   # 上一次打印速度指令的时间

        self.open_serial_port() ##尝试打开串口，如果失败会在日志中看到错误，并且会定时重试打开。

        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_callback, 10) ##订阅 /cmd_vel 话题，消息类型是 geometry_msgs/msg/Twist，回调函数是 cmd_vel_callback，队列长度 10。
        self.create_timer(0.02, self.timer_callback) ##创建一个定时器，每 20 ms 调用一次 timer_callback，用来处理串口接收和数据发布。
        self.create_timer( 
            RECONNECT_INTERVAL_SEC,
            self.reconnect_timer_callback,
        )##创建一个定时器，每 RECONNECT_INTERVAL_SEC 秒调用一次 reconnect_timer_callback，用来检查串口连接状态，如果断开了就尝试重连。
        self.publish_status("initialized")
        self.get_logger().info("esp32_bridge_node initialized successfully")

    def publish_status(self, text):
        """发布 ESP32 连接状态。

        参数：
            text：状态文本，例如 initialized、serial_connected。
        """
        status_msg = String()
        status_msg.data = text
        self.status_pub.publish(status_msg)

    def open_serial_port(self):
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
        if serial is None:
            return

        if self.serial_port is None:
            self.open_serial_port()

    def can_print_now(self, last_print_time):
        """判断当前是否允许打印一条普通调试日志。

        参数：
            last_print_time：上一条同类日志的打印时间，单位秒。

        返回：
            tuple：(是否允许打印，本次记录的打印时间)。
        """
        enable_print = self.get_parameter("enable_print").value
        if not enable_print:
            return False, last_print_time

        now = time.monotonic()
        if now - last_print_time < PRINT_INTERVAL_SEC:
            return False, last_print_time
        return True, now

    def timer_callback(self):
        """定时器回调函数，负责从串口接收数据，解析成消息，并且根据功能码打印日志或者发布 ROS2 话题。
        """
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
            self.print_serial_message(func, payload)

    def print_serial_message(self, func, payload):
        if func == FUNC_ENCODER:
            self.print_encoder_message(payload)
            return

        if func == FUNC_IMU:
            self.print_imu_message(payload)
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

    def print_encoder_message(self, payload):##
        """"
        解析编码器数据，发布里程计，并且根据参数决定是否打印日志。"""
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
        """根据左右轮速度发布 /odom。

        参数：
            left_speed_cm_s：左驱动轮实际速度，单位 cm/s。
            right_speed_cm_s：右驱动轮实际速度，单位 cm/s。
        """
        now = self.get_clock().now()

        if self.last_odom_time is None:
            dt = 0.0
        else:
            dt = (now - self.last_odom_time).nanoseconds / 1000000000.0

        self.last_odom_time = now

        left_speed_m_s = left_speed_cm_s / 100.0
        right_speed_m_s = right_speed_cm_s / 100.0
        linear_speed = (left_speed_m_s + right_speed_m_s) / 2.0
        angular_speed = (right_speed_m_s - left_speed_m_s) / self.wheel_base_m

        if dt > 0.0:
            self.odom_x += linear_speed * math.cos(self.odom_yaw) * dt
            self.odom_y += linear_speed * math.sin(self.odom_yaw) * dt
            self.odom_yaw += angular_speed * dt

        qx, qy, qz, qw = build_quaternion_from_yaw(self.odom_yaw)

        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        odom_msg.pose.pose.position.x = self.odom_x
        odom_msg.pose.pose.position.y = self.odom_y
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw
        odom_msg.twist.twist.linear.x = linear_speed
        odom_msg.twist.twist.angular.z = angular_speed
        self.odom_pub.publish(odom_msg)

    def print_imu_message(self, payload):
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

        self.publish_imu(
            accel_x,
            accel_y,
            accel_z,
            gyro_x,
            gyro_y,
            gyro_z,
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

    def publish_imu(
        self,
        accel_x,
        accel_y,
        accel_z,
        gyro_x,
        gyro_y,
        gyro_z,
    ):
        """发布 /imu/data。

        参数：
            accel_x：MPU6050 原始 X 轴加速度。
            accel_y：MPU6050 原始 Y 轴加速度。
            accel_z：MPU6050 原始 Z 轴加速度。
            gyro_x：MPU6050 原始 X 轴角速度。
            gyro_y：MPU6050 原始 Y 轴角速度。
            gyro_z：MPU6050 原始 Z 轴角速度。
        """
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.orientation_covariance[0] = -1.0

        # 当前先发布 MPU6050 原始数值，不在上位机重复做姿态解算。
        imu_msg.linear_acceleration.x = float(accel_x)
        imu_msg.linear_acceleration.y = float(accel_y)
        imu_msg.linear_acceleration.z = float(accel_z)
        imu_msg.angular_velocity.x = float(gyro_x)
        imu_msg.angular_velocity.y = float(gyro_y)
        imu_msg.angular_velocity.z = float(gyro_z)
        self.imu_pub.publish(imu_msg)

    def cmd_vel_callback(self, msg):
        """处理 /cmd_vel，把线速度和角速度转换成左右轮速度。

        参数：
            msg：geometry_msgs/msg/Twist，来自键盘遥控或 Nav2。
        """
        linear_speed = msg.linear.x
        angular_speed = msg.angular.z

        left_speed_m_s = linear_speed - angular_speed * self.wheel_base_m / 2.0
        right_speed_m_s = (
            linear_speed + angular_speed * self.wheel_base_m / 2.0
        )

        left_speed_cm_s = limit_wheel_speed(left_speed_m_s * 100.0)
        right_speed_cm_s = limit_wheel_speed(right_speed_m_s * 100.0)

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
