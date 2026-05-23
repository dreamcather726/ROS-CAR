#include "serial_sender.h"
#include <math.h>

// 计算 CRC16。
// 这里使用串口和工业协议里很常见的 CRC16-Modbus。
// 初学时只要记住：按字节循环，再按位循环即可。
uint16_t ss_crc16(const uint8_t bytes[], size_t len) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < len; i++) {
    crc ^= bytes[i];

    for (uint8_t bit = 0; bit < 8; bit++) {
      if ((crc & 0x0001) != 0) {
        crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
      } else {
        crc = static_cast<uint16_t>(crc >> 1);
      }
    }
  }

  return crc;
}

void ss_pack_int_le(int32_t value, uint8_t byte_count, uint8_t out[]) {
  if (out == nullptr) return;
  if (byte_count == 0) return;

  uint32_t raw = static_cast<uint32_t>(value);
  for (uint8_t i = 0; i < byte_count; i++) {
    out[i] = static_cast<uint8_t>((raw >> (8 * i)) & 0xFFU);
  }
}

void ss_send_encoder_counts(HardwareSerial &serial,
                            uint8_t func,
                            int32_t left_count,
                            int32_t right_count,
                            bool flush_after_send) {
  uint8_t payload6[6];
  uint8_t frame[SERIAL_SENDER_MAX_FRAME_SIZE];
  ss_pack_int_le(left_count, 3, &payload6[0]);
  ss_pack_int_le(right_count, 3, &payload6[3]);

  uint8_t frame_len = ss_build_frame(func, payload6, 6, frame);
  if (frame_len == 0) {
    return;
  }

  serial.write(frame, frame_len);
  if (flush_after_send) {
    serial.flush();
  }
}

void ss_send_ultrasonic_distance_cm_x10(HardwareSerial &serial,
                                        uint8_t func,
                                        float distance_cm,
                                        bool flush_after_send) {
  int16_t distance_x10 = -1;
  if (distance_cm >= 0.0f) {
    long scaled = lroundf(distance_cm * 10.0f);
    if (scaled > 32767L) {
      scaled = 32767L;
    }
    distance_x10 = static_cast<int16_t>(scaled);
  }

  uint8_t payload2[2];
  uint8_t frame[SERIAL_SENDER_MAX_FRAME_SIZE];
  ss_pack_int_le(distance_x10, 2, payload2);

  uint8_t frame_len = ss_build_frame(func, payload2, 2, frame);
  if (frame_len == 0) {
    return;
  }

  serial.write(frame, frame_len);
  if (flush_after_send) {
    serial.flush();
  }
}

// 构建发送数据帧。
// 新格式：
// [0]  帧头
// [1]  长度 = func + payload 的总字节数
// [2]  功能码
// [3]~[...] payload
// [最后3] CRC低字节
// [最后2] CRC高字节
// [最后1] 帧尾
uint8_t ss_build_frame(uint8_t func,// 功能码
                       const uint8_t payload[],// 数据区
                       uint8_t payload_len,// 数据区长度
                       uint8_t out[SERIAL_SENDER_MAX_FRAME_SIZE]) {
  if (payload_len > SERIAL_SENDER_MAX_PAYLOAD_SIZE) {
    return 0;
  }

  uint8_t data_len = static_cast<uint8_t>(1 + payload_len);
  uint8_t frame_len = static_cast<uint8_t>(1 + 1 + data_len + SERIAL_SENDER_CRC_SIZE + 1);

  out[0] = FRAME_HEADER;
  out[1] = data_len;
  out[2] = func;

  for (uint8_t i = 0; i < payload_len; i++) {
    out[3 + i] = payload[i];
  }

  // CRC 只校验“长度 + 数据区”，不包含帧头和帧尾。
  // 这样接收端在找到帧头后，只需要验证中间有效内容是否正确。
  uint8_t crc_input[1 + SERIAL_SENDER_MAX_DATA_SIZE];
  crc_input[0] = out[1];
  for (uint8_t i = 0; i < data_len; i++) {
    crc_input[1 + i] = out[2 + i];
  }

  uint16_t crc = ss_crc16(crc_input, 1 + data_len);
  out[3 + payload_len] = static_cast<uint8_t>(crc & 0xFF);
  out[4 + payload_len] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  out[5 + payload_len] = FRAME_TAIL;
  return frame_len;
}

// 发送数据帧。
// 这里先组帧，再一次性写入串口。
void ss_send(HardwareSerial &serial,
             uint8_t func,
             const uint8_t payload[],
             uint8_t payload_len,
             bool flush_after_send) {
  if (payload_len > SERIAL_SENDER_MAX_PAYLOAD_SIZE) {
    return;
  }
  if (payload == nullptr && payload_len != 0) {
    return;
  }
  uint8_t frame[SERIAL_SENDER_MAX_FRAME_SIZE];
  uint8_t frame_len = ss_build_frame(func, payload, payload_len, frame);
  if (frame_len == 0) {
    return;
  }
  serial.write(frame, frame_len);
  if (flush_after_send) {
    serial.flush();
  }
}
