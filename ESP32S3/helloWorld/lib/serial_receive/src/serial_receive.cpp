#include "serial_receive.h"

static constexpr uint8_t SERIAL_RECEIVE_HEADER = 0xAA;
static constexpr uint8_t SERIAL_RECEIVE_TAIL = 0xBB;

static constexpr size_t SERIAL_RECEIVE_MAX_DATA_SIZE = 1 + SERIAL_RECEIVE_MAX_PAYLOAD_SIZE;
static constexpr size_t SERIAL_RECEIVE_MAX_FRAME_SIZE = 1 + 1 + SERIAL_RECEIVE_MAX_DATA_SIZE + 2 + 1;

static uint8_t g_rx_frame[SERIAL_RECEIVE_MAX_FRAME_SIZE];
static uint8_t g_rx_pos = 0;
static uint8_t g_expect_len = 0;
static uint8_t g_data_len = 0;

static bool g_has_frame = false;
static SerialReceiveFrame g_last_frame = {0, 0, {0}};
// 重置接收状态
static uint16_t crc16_modbus(const uint8_t bytes[], size_t len)
{
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
// 重置接收状态
static void rx_reset_state()
{
  g_rx_pos = 0;
  g_expect_len = 0;
  g_data_len = 0;
}
// 尝试完成一帧数据
static bool rx_try_finish_frame()
{
  if (g_expect_len == 0 || g_rx_pos < g_expect_len) {
    return false;
  }
  // Serial.print("rx frame (len=");
  // Serial.print(g_expect_len);
  // Serial.print("): ");
  // for (uint8_t i = 0; i < g_expect_len; i++) {
  //   if (g_rx_frame[i] < 16) Serial.print('0');
  //   // Serial.print(g_rx_frame[i], HEX);
  //   if (i + 1 < g_expect_len) Serial.print(' ');
  // }
  // Serial.println();
  // 校验尾部
  const uint8_t tail = g_rx_frame[g_expect_len - 1];
  if (tail != SERIAL_RECEIVE_TAIL) {
    Serial.print("bad tail: 0x");
    if (tail < 16) Serial.print('0');
    Serial.println(tail, HEX);
    rx_reset_state();
    return true;
  }

  const uint8_t *data = &g_rx_frame[2];

  // -------- 修正CRC读取：小端（低字节在前）--------
  uint8_t crc_low  = g_rx_frame[2 + g_data_len];
  uint8_t crc_high = g_rx_frame[3 + g_data_len];
  const uint16_t recv_crc = (static_cast<uint16_t>(crc_high) << 8) | crc_low;
  // ----------------------------------------------------

  uint8_t crc_input[1 + SERIAL_RECEIVE_MAX_DATA_SIZE];
  crc_input[0] = g_data_len;
  for (uint8_t i = 0; i < g_data_len; i++) {
    crc_input[1 + i] = data[i];
  }
  // 计算 CRC16
  const uint16_t calc_crc = crc16_modbus(crc_input, 1 + g_data_len);
  // 校验 CRC16
  // Serial.printf("calc_crc: 0x%04X, recv_crc: 0x%04X\n", calc_crc, recv_crc);
  if (calc_crc != recv_crc) {// 如果校验失败
    // Serial.println("CRC16 校验失败");
    rx_reset_state();
    return true;
  }
  // Serial.println("CRC16 校验通过");

  const uint8_t func = data[0];
  const uint8_t payload_len = static_cast<uint8_t>(g_data_len - 1);
  g_last_frame.func = func;
  g_last_frame.payload_len = payload_len;
  const uint8_t copy_len =
      (payload_len > SERIAL_RECEIVE_MAX_PAYLOAD_SIZE) ? SERIAL_RECEIVE_MAX_PAYLOAD_SIZE : payload_len;
  for (uint8_t i = 0; i < copy_len; i++) {
    g_last_frame.payload[i] = data[1 + i];
  }
  g_has_frame = true;

  // Serial.print("func=0x");
  // if (func < 16) Serial.print('0');
  // Serial.print(func, HEX);
  // Serial.print(" payload_len=");
  // Serial.println(payload_len);
  // if (payload_len > 0) {
  //   Serial.print("payload: ");
  //   for (uint8_t i = 0; i < copy_len; i++) {
  //     const uint8_t v = g_last_frame.payload[i];
  //     if (v < 16) Serial.print('0');
  //     Serial.print(v, HEX);
  //     if (i + 1 < copy_len) Serial.print(' ');
  //   }
  //   Serial.println();
  // }
  
  rx_reset_state();
  return true;
}
// 更新接收状态
void serial_receive_update(Stream &serial)
{
  while (serial.available() > 0) {
    const int b = serial.read();
    
    if (b < 0) {
      break;
    }
    const uint8_t ub = static_cast<uint8_t>(b);

    if (g_rx_pos == 0) {
      if (ub != SERIAL_RECEIVE_HEADER) {
        continue;
      }
      // Serial.println("rx header 0xAA");
      g_rx_frame[0] = ub;
      g_rx_pos = 1;
      g_expect_len = 0;
      g_data_len = 0;
      continue;
    }

    if (g_rx_pos >= SERIAL_RECEIVE_MAX_FRAME_SIZE) {
      rx_reset_state();
      continue;
    }
    // 保存数据
    g_rx_frame[g_rx_pos++] = ub;

    if (g_rx_pos == 2) {
      g_data_len = g_rx_frame[1];
      // Serial.print("rx data_len=");
      // Serial.println(g_data_len);  
      // 数据长度必须在 1 到 SERIAL_RECEIVE_MAX_PAYLOAD_SIZE 之间
      if (g_data_len < 1 || g_data_len > SERIAL_RECEIVE_MAX_DATA_SIZE) {
        // Serial.println("bad data_len, reset");
        rx_reset_state();
        continue;
      }
      // 计算期望长度
      g_expect_len = static_cast<uint8_t>(1 + 1 + g_data_len + 2 + 1);
      // if (g_expect_len > SERIAL_RECEIVE_MAX_FRAME_SIZE) {
      //   Serial.println("bad frame_len, reset");
      //   rx_reset_state();
      //   continue;
      // }
    }

    rx_try_finish_frame();
  }
}

bool serial_receive_take_frame(SerialReceiveFrame *out)
{
  if (!out) return false;
  if (!g_has_frame) return false;
  *out = g_last_frame;
  g_has_frame = false;
  return true;
}
