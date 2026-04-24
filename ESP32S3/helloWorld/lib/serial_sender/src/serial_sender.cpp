#include "serial_sender.h"

uint8_t ss_checksum8(const uint8_t *bytes, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return sum;
}

void ss_build_frame(uint8_t func,
                    const uint8_t data[SERIAL_SENDER_DATA_SIZE],
                    uint8_t out[SERIAL_SENDER_FRAME_SIZE]) {
  out[0] = SERIAL_SENDER_HEADER;
  out[1] = func;
  for (size_t i = 0; i < SERIAL_SENDER_DATA_SIZE; i++) {
    out[2 + i] = data ? data[i] : 0x00;
  }
  out[8] = ss_checksum8(out, 8);
  out[9] = SERIAL_SENDER_TAIL;
}
// 发送数据帧
void ss_send(HardwareSerial &serial,
             uint8_t func,
             const uint8_t data[SERIAL_SENDER_DATA_SIZE],
             bool flush_after_send) {
  uint8_t frame[SERIAL_SENDER_FRAME_SIZE];
  ss_build_frame(func, data, frame);
  serial.write(frame, SERIAL_SENDER_FRAME_SIZE);
  if (flush_after_send) {
    serial.flush();
  }
}

void ss_send6(HardwareSerial &serial,
              uint8_t func,
              uint8_t d0,
              uint8_t d1,
              uint8_t d2,
              uint8_t d3,
              uint8_t d4,
              uint8_t d5,
              bool flush_after_send) {
  const uint8_t data[SERIAL_SENDER_DATA_SIZE] = {d0, d1, d2, d3, d4, d5};
  ss_send(serial, func, data, flush_after_send);
}
