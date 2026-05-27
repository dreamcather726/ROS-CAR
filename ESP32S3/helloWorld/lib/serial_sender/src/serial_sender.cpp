#include "serial_sender.h"
#include <math.h>

static inline int16_t ss_clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

static inline float ss_wrap_deg_360(float deg) {
  float x = fmodf(deg, 360.0f);
  if (x < 0.0f) x += 360.0f;
  return x;
}

static inline void write_le16(uint8_t *buf, uint16_t val) {
  buf[0] = static_cast<uint8_t>(val & 0xFF);
  buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
}

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

void ss_pack_mpu6050_raw_i16_le(int16_t ax,
                                int16_t ay,
                                int16_t az,
                                int16_t gx,
                                int16_t gy,
                                int16_t gz,
                                uint8_t out12[12]) {
  if (out12 == nullptr) return;

  ss_pack_int_le(ax, 2, &out12[0]);
  ss_pack_int_le(ay, 2, &out12[2]);
  ss_pack_int_le(az, 2, &out12[4]);

  ss_pack_int_le(gx, 2, &out12[6]);
  ss_pack_int_le(gy, 2, &out12[8]);
  ss_pack_int_le(gz, 2, &out12[10]);
}

void ss_pack_rpy_deg_x10_i16_le(float roll_deg,
                                float pitch_deg,
                                float yaw_deg,
                                uint8_t out6[6]) {
  if (out6 == nullptr) return;

  const float r_deg = roll_deg;
  const float p_deg = pitch_deg;
  const float y_deg = ss_wrap_deg_360(yaw_deg);

  const int16_t r = ss_clamp_i16(static_cast<int32_t>(lroundf(r_deg * 10.0f)));
  const int16_t p = ss_clamp_i16(static_cast<int32_t>(lroundf(p_deg * 10.0f)));
  const int16_t y = ss_clamp_i16(static_cast<int32_t>(lroundf(y_deg * 10.0f)));

  ss_pack_int_le(r, 2, &out6[0]);
  ss_pack_int_le(p, 2, &out6[2]);
  ss_pack_int_le(y, 2, &out6[4]);
}

void ss_pack_mpu6050_bundle(int16_t ax,
                            int16_t ay,
                            int16_t az,
                            int16_t gx,
                            int16_t gy,
                            int16_t gz,
                            float roll_deg,
                            float pitch_deg,
                            float yaw_deg,
                            uint8_t out18[18]) {
  if (out18 == nullptr) return;

  ss_pack_mpu6050_raw_i16_le(ax, ay, az, gx, gy, gz, &out18[0]);
  ss_pack_rpy_deg_x10_i16_le(roll_deg, pitch_deg, yaw_deg, &out18[12]);
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
  write_le16(&out[3 + payload_len], crc);
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
