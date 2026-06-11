"""Serial protocol helpers for the ESP32 bridge."""

import struct

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


def receive_serial_messages(serial_port, receive_buffer, logger):
    """Read bytes from serial and return complete protocol messages.

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


def parse_serial_frame(frame, logger):
    """Check one complete serial frame and return func and payload."""
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


def calculate_crc16(data_bytes):
    """Calculate CRC16-Modbus checksum."""
    crc = 0xFFFF
    for one_byte in data_bytes:
        crc ^= one_byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def read_int16_le(payload, offset):
    """Read a little-endian signed int16 from payload."""
    return struct.unpack_from("<h", payload, offset)[0]


def read_int24_le(payload, offset):
    """Read a little-endian signed int24 from payload."""
    parsed_value = (
        payload[offset]
        | (payload[offset + 1] << 8)
        | (payload[offset + 2] << 16)
    )
    if parsed_value & 0x800000:
        parsed_value -= 0x1000000
    return parsed_value


def build_serial_frame(func, payload):
    """Build one complete ESP32 serial frame."""
    frame_data = bytes([func]) + payload
    data_length = len(frame_data)
    crc = calculate_crc16(bytes([data_length]) + frame_data)

    return (
        bytes([FRAME_HEADER, data_length])
        + frame_data
        + bytes([crc & 0xFF, (crc >> 8) & 0xFF, FRAME_TAIL])
    )
