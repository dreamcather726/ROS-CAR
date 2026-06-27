#pragma once

#include <Arduino.h>

static constexpr size_t SERIAL_RECEIVE_MAX_PAYLOAD_SIZE = 32;

struct SerialReceiveFrame {
  uint8_t func;
  uint8_t payload_len;
  uint8_t payload[SERIAL_RECEIVE_MAX_PAYLOAD_SIZE];
};

void serial_receive_update(Stream &serial);// 更新接收状态
bool serial_receive_take_frame(SerialReceiveFrame *out);
