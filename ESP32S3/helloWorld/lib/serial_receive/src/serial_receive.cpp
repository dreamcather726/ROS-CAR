#include "serial_receive.h"

static uint8_t g_rx_frame[SERIAL_RECEIVE_MAX_FRAME_SIZE];
static uint8_t g_rx_pos = 0;
static uint8_t g_expect_len = 0;
static uint8_t g_data_len = 0;

static bool g_has_frame = false;
static SerialReceiveFrame g_last_frame = {0, 0, {0}, 0};

static uint32_t g_last_rx_ms = 0;
static SerialReceiveStats g_stats = {0, 0, 0, 0};

static Stream *g_debug = nullptr;
static bool g_debug_print_bytes = false;
static bool g_debug_print_frames = true;
static bool g_debug_print_errors = true;

static void dbg_hex2(uint8_t v)
{
  if (!g_debug) return;
  if (v < 16) g_debug->print('0');
  g_debug->print(v, HEX);
}

static void dbg_print_frame(const uint8_t frame[], uint8_t len)
{
  if (!g_debug || !g_debug_print_frames) return;
  g_debug->print("rx frame: ");
  for (uint8_t i = 0; i < len; i++) {
    dbg_hex2(frame[i]);
    if (i + 1 < len) g_debug->print(' ');
  }
  g_debug->println();
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

/**
 * @brief 重置接收统计信息
 * 
 */
void serial_receive_reset()
{
  g_rx_pos = 0;
  g_expect_len = 0;
  g_data_len = 0;

  g_has_frame = false;
  g_last_frame = {0, 0, {0}, 0};

  g_last_rx_ms = 0;
  g_stats = {0, 0, 0, 0};
}

void serial_receive_set_debug(Stream *debug_stream)
{
  g_debug = debug_stream;
}

void serial_receive_set_debug_flags(bool print_bytes, bool print_frames, bool print_errors)
{
  g_debug_print_bytes = print_bytes;
  g_debug_print_frames = print_frames;
  g_debug_print_errors = print_errors;
}

static void on_frame(uint8_t func, const uint8_t payload[], uint8_t payload_len)
{
  const uint32_t now_ms = millis();
  g_last_rx_ms = now_ms;
  g_last_frame.func = func;
  g_last_frame.payload_len = payload_len;
  for (uint8_t i = 0; i < payload_len; i++) {
    g_last_frame.payload[i] = payload[i];
  }
  g_last_frame.rx_ms = now_ms;
  g_has_frame = true;
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
    if (g_debug && g_debug_print_bytes) {
      g_debug->print("rx byte 0x");
      dbg_hex2(ub);
      g_debug->print(" pos=");
      g_debug->println(static_cast<unsigned>(g_rx_pos));
    }

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

    g_rx_frame[g_rx_pos++] = ub;
    if (g_rx_pos == 2) {
      g_data_len = g_rx_frame[1];
      if (g_data_len < 1 || g_data_len > SERIAL_RECEIVE_MAX_DATA_SIZE) {
        g_stats.unknown_func++;
        g_rx_pos = 0;
        continue;
      }
      g_expect_len = static_cast<uint8_t>(1 + 1 + g_data_len + 2 + 1);
    }

    if (g_expect_len == 0 || g_rx_pos < g_expect_len) {
      continue;
    }

    dbg_print_frame(g_rx_frame, g_expect_len);

    const uint8_t tail = g_rx_frame[g_expect_len - 1];
    if (tail != SERIAL_RECEIVE_TAIL) {
      g_stats.bad_tail++;
      if (g_debug && g_debug_print_errors) {
        g_debug->print("rx bad tail got 0x");
        dbg_hex2(tail);
        g_debug->println();
      }
      g_rx_pos = 0;
      g_expect_len = 0;
      g_data_len = 0;
      continue;
    }

    const uint8_t *data = &g_rx_frame[2];
    const uint16_t recv_crc = static_cast<uint16_t>(g_rx_frame[2 + g_data_len]) |
                              (static_cast<uint16_t>(g_rx_frame[3 + g_data_len]) << 8);
    uint8_t crc_input[1 + SERIAL_RECEIVE_MAX_DATA_SIZE];
    crc_input[0] = g_data_len;
    for (uint8_t i = 0; i < g_data_len; i++) {
      crc_input[1 + i] = data[i];
    }
    const uint16_t calc_crc = crc16_modbus(crc_input, 1 + g_data_len);
    if (calc_crc != recv_crc) {
      g_stats.checksum_fail++;
      if (g_debug && g_debug_print_errors) {
        g_debug->print("rx crc fail calc=0x");
        dbg_hex2(static_cast<uint8_t>(calc_crc & 0xFF));
        dbg_hex2(static_cast<uint8_t>((calc_crc >> 8) & 0xFF));
        g_debug->print(" recv=0x");
        dbg_hex2(static_cast<uint8_t>(recv_crc & 0xFF));
        dbg_hex2(static_cast<uint8_t>((recv_crc >> 8) & 0xFF));
        g_debug->println();
      }
      g_rx_pos = 0;
      g_expect_len = 0;
      g_data_len = 0;
      continue;
    }

    g_stats.frames_ok++;
    const uint8_t func = data[0];
    const uint8_t payload_len = static_cast<uint8_t>(g_data_len - 1);
    on_frame(func, &data[1], payload_len);

    g_rx_pos = 0;
    g_expect_len = 0;
    g_data_len = 0;
  }
}

bool serial_receive_take_frame(SerialReceiveFrame *out)
{
  if (!out) {
    return false;
  }
  if (!g_has_frame) {
    return false;
  }
  *out = g_last_frame;
  g_has_frame = false;
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
