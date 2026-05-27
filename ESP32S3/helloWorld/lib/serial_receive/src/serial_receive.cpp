#include "serial_receive.h"

static constexpr uint8_t SERIAL_RECEIVE_HEADER = 0xAA;
static constexpr uint8_t SERIAL_RECEIVE_TAIL = 0xBB;

static constexpr size_t SERIAL_RECEIVE_MAX_DATA_SIZE = 1 + SERIAL_RECEIVE_MAX_PAYLOAD_SIZE;
static constexpr size_t SERIAL_RECEIVE_MAX_FRAME_SIZE = 1 + 1 + SERIAL_RECEIVE_MAX_DATA_SIZE + 2 + 1;

static uint8_t g_rx_frame[SERIAL_RECEIVE_MAX_FRAME_SIZE];
static uint8_t g_rx_pos = 0;
static uint8_t g_expect_len = 0;
static uint8_t g_data_len = 0;

static constexpr uint32_t SERIAL_RECEIVE_RX_TIMEOUT_MS = 30;
static uint32_t g_last_byte_ms = 0;

static constexpr uint8_t SERIAL_RECEIVE_QUEUE_SIZE = 4;
static SerialReceiveFrame g_queue[SERIAL_RECEIVE_QUEUE_SIZE];
static uint8_t g_q_head = 0;
static uint8_t g_q_tail = 0;
static uint8_t g_q_count = 0;

static void queue_push(const SerialReceiveFrame &f)
{
  if (g_q_count >= SERIAL_RECEIVE_QUEUE_SIZE) {
    g_q_tail = static_cast<uint8_t>((g_q_tail + 1) % SERIAL_RECEIVE_QUEUE_SIZE);
    g_q_count--;
  }
  g_queue[g_q_head] = f;
  g_q_head = static_cast<uint8_t>((g_q_head + 1) % SERIAL_RECEIVE_QUEUE_SIZE);
  g_q_count++;
}

static bool queue_pop(SerialReceiveFrame *out)
{
  if (!out) return false;
  if (g_q_count == 0) return false;
  *out = g_queue[g_q_tail];
  g_q_tail = static_cast<uint8_t>((g_q_tail + 1) % SERIAL_RECEIVE_QUEUE_SIZE);
  g_q_count--;
  return true;
}

static inline uint16_t read_le16(const uint8_t *buf) {
  return static_cast<uint16_t>(buf[0]) | (static_cast<uint16_t>(buf[1]) << 8);
}

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
  g_last_byte_ms = 0;
}
// 尝试完成一帧数据
static bool rx_try_finish_frame()
{
  if (g_expect_len == 0 || g_rx_pos < g_expect_len) {
    return false;
  }

  const uint8_t tail = g_rx_frame[g_expect_len - 1];
  if (tail != SERIAL_RECEIVE_TAIL) {
    rx_reset_state();
    return true;
  }

  const uint8_t *data = &g_rx_frame[2];

  const uint16_t recv_crc = read_le16(&g_rx_frame[2 + g_data_len]);

  uint8_t crc_input[1 + SERIAL_RECEIVE_MAX_DATA_SIZE];
  crc_input[0] = g_data_len;
  for (uint8_t i = 0; i < g_data_len; i++) {
    crc_input[1 + i] = data[i];
  }
  // 计算 CRC16
  const uint16_t calc_crc = crc16_modbus(crc_input, 1 + g_data_len);

  if (calc_crc != recv_crc) {// 如果校验失败

    rx_reset_state();
    return true;
  }


  const uint8_t func = data[0];
  const uint8_t payload_len = static_cast<uint8_t>(g_data_len - 1);
  SerialReceiveFrame f = {0, 0, {0}};
  f.func = func;
  f.payload_len = payload_len;
  const uint8_t copy_len = (payload_len > SERIAL_RECEIVE_MAX_PAYLOAD_SIZE)
                               ? SERIAL_RECEIVE_MAX_PAYLOAD_SIZE
                               : payload_len;
  for (uint8_t i = 0; i < copy_len; i++) {
    f.payload[i] = data[1 + i];
  }
  queue_push(f);

  rx_reset_state();
  return true;
}
// 更新接收状态
void serial_receive_update(Stream &serial)
{
  const uint32_t now_ms = millis();
  if (g_rx_pos != 0 && g_last_byte_ms != 0 && (now_ms - g_last_byte_ms) > SERIAL_RECEIVE_RX_TIMEOUT_MS) {
    rx_reset_state();
  }

  while (serial.available() > 0) {
    const int b = serial.read();
    
    if (b < 0) {
      break;
    }
    const uint8_t ub = static_cast<uint8_t>(b);
    g_last_byte_ms = millis();

    if (g_rx_pos == 0) {
      if (ub != SERIAL_RECEIVE_HEADER) {
        continue;
      }
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
 
      // 数据长度必须在 1 到 SERIAL_RECEIVE_MAX_PAYLOAD_SIZE 之间
      if (g_data_len < 1 || g_data_len > SERIAL_RECEIVE_MAX_DATA_SIZE) {
 
        rx_reset_state();
        continue;
      }
      // 计算期望长度
      g_expect_len = static_cast<uint8_t>(1 + 1 + g_data_len + 2 + 1);
     
    }

    rx_try_finish_frame();
  }
}

bool serial_receive_take_frame(SerialReceiveFrame *out)
{
  return queue_pop(out);
}
