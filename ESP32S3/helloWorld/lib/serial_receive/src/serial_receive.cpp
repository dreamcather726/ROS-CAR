#include "serial_receive.h"

static uint8_t g_rx_frame[SERIAL_RECEIVE_FRAME_SIZE];
static size_t g_rx_pos = 0;

static bool g_has_speed = false;
static float g_speed_left_cm_s = 0.0f;
static float g_speed_right_cm_s = 0.0f;
static uint32_t g_speed_rx_ms = 0;

static bool g_has_target_counts = false;
static int32_t g_target_left_counts = 0;
static int32_t g_target_right_counts = 0;
static uint32_t g_target_counts_rx_ms = 0;

static uint32_t g_last_rx_ms = 0;
static SerialReceiveStats g_stats = {0, 0, 0, 0};

static uint8_t checksum8(const uint8_t *bytes, size_t len)
{
  uint8_t sum = 0;
  for (size_t i = 0; i < len; i++) {
    sum = static_cast<uint8_t>(sum + bytes[i]);
  }
  return sum;
}

static int16_t read_i16_le(const uint8_t *p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

static int32_t read_i24_le(const uint8_t *p)
{
  int32_t v = static_cast<int32_t>(p[0]) | (static_cast<int32_t>(p[1]) << 8) |
              (static_cast<int32_t>(p[2]) << 16);
  if (v & 0x800000) {
    v |= static_cast<int32_t>(0xFF000000);
  }
  return v;
}

/**
 * @brief 重置接收统计信息
 * 
 */
void serial_receive_reset()
{
  g_rx_pos = 0;
  g_has_speed = false;
  g_speed_left_cm_s = 0.0f;
  g_speed_right_cm_s = 0.0f;
  g_speed_rx_ms = 0;

  g_has_target_counts = false;
  g_target_left_counts = 0;
  g_target_right_counts = 0;
  g_target_counts_rx_ms = 0;

  g_last_rx_ms = 0;
  g_stats = {0, 0, 0, 0};
}
/**
 * @brief 处理接收到的帧
 * 
 * @param func 函数码
 * @param payload 数据负载
 */

static void on_frame(uint8_t func, const uint8_t payload[SERIAL_RECEIVE_DATA_SIZE])
{
  const uint32_t now_ms = millis();
  g_last_rx_ms = now_ms;

  if (func == SERIAL_RECEIVE_FUNC_WHEEL_SPEED) {
    const int16_t l = read_i16_le(&payload[0]);
    const int16_t r = read_i16_le(&payload[2]);
    g_speed_left_cm_s = static_cast<float>(l) / 100.0f;
    g_speed_right_cm_s = static_cast<float>(r) / 100.0f;
    g_speed_rx_ms = now_ms;
    g_has_speed = true;
    return;
  }

  if (func == SERIAL_RECEIVE_FUNC_WHEEL_TARGET_COUNTS) {
    g_target_left_counts = read_i24_le(&payload[0]);
    g_target_right_counts = read_i24_le(&payload[3]);
    g_target_counts_rx_ms = now_ms;
    g_has_target_counts = true;
    return;
  }

  g_stats.unknown_func++;
}

/**
 * @brief 更新接收统计信息
 * 
 * @param serial 串口对象
 */
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
      g_rx_frame[0] = ub;
      g_rx_pos = 1;
      continue;
    }

    if (ub == SERIAL_RECEIVE_HEADER) {
      g_rx_frame[0] = ub;
      g_rx_pos = 1;
      continue;
    }

    g_rx_frame[g_rx_pos++] = ub;
    if (g_rx_pos < SERIAL_RECEIVE_FRAME_SIZE) {
      continue;
    }

    g_rx_pos = 0;

    if (g_rx_frame[SERIAL_RECEIVE_FRAME_SIZE - 1] != SERIAL_RECEIVE_TAIL) {
      g_stats.bad_tail++;
      continue;
    }

    const uint8_t calc = checksum8(g_rx_frame, 8);
    const uint8_t recv = g_rx_frame[8];
    if (calc != recv) {
      g_stats.checksum_fail++;
      continue;
    }

    g_stats.frames_ok++;
    on_frame(g_rx_frame[1], &g_rx_frame[2]);
  }
}

bool serial_receive_take_wheel_speed_cm_s(float *left_cm_s, float *right_cm_s, uint32_t *rx_ms)
{
  if (!g_has_speed) {
    return false;
  }
  if (left_cm_s) *left_cm_s = g_speed_left_cm_s;
  if (right_cm_s) *right_cm_s = g_speed_right_cm_s;
  if (rx_ms) *rx_ms = g_speed_rx_ms;
  g_has_speed = false;
  return true;
}

bool serial_receive_take_wheel_target_counts(int32_t *left_counts,
                                            int32_t *right_counts,
                                            uint32_t *rx_ms)
{
  if (!g_has_target_counts) {
    return false;
  }
  if (left_counts) *left_counts = g_target_left_counts;
  if (right_counts) *right_counts = g_target_right_counts;
  if (rx_ms) *rx_ms = g_target_counts_rx_ms;
  g_has_target_counts = false;
  return true;
}

uint32_t serial_receive_last_rx_ms()
{
  return g_last_rx_ms;
}

SerialReceiveStats serial_receive_stats()
{
  return g_stats;
}
