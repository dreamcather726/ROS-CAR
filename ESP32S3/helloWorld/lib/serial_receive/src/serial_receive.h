#pragma once

#include <Arduino.h>

static constexpr uint8_t SERIAL_RECEIVE_HEADER = 0xAA;// 0xAA: 头部
static constexpr uint8_t SERIAL_RECEIVE_TAIL = 0xBB;// 0xBB: 尾部
static constexpr size_t SERIAL_RECEIVE_MAX_PAYLOAD_SIZE = 32;
static constexpr size_t SERIAL_RECEIVE_MAX_DATA_SIZE = 1 + SERIAL_RECEIVE_MAX_PAYLOAD_SIZE;
static constexpr size_t SERIAL_RECEIVE_MAX_FRAME_SIZE = 1 + 1 + SERIAL_RECEIVE_MAX_DATA_SIZE + 2 + 1;
// 接收统计信息
// 接收统计信息
/**
 * @brief 接收统计信息
 * 
 */
struct SerialReceiveStats {
  uint32_t frames_ok;
  uint32_t bad_tail;
  uint32_t checksum_fail;
  uint32_t unknown_func;
};

struct SerialReceiveFrame {
  uint8_t func;
  uint8_t payload_len;
  uint8_t payload[SERIAL_RECEIVE_MAX_PAYLOAD_SIZE];
  uint32_t rx_ms;
};
// 重置接收统计信息
/**
 * @brief 重置接收统计信息
 * 
 */
void serial_receive_reset();
// 更新接收统计信息
/**
 * @brief 更新接收统计信息
 * 
 * @param serial 串口对象
 */
void serial_receive_update(Stream &serial);
void serial_receive_set_debug(Stream *debug_stream);
void serial_receive_set_debug_flags(bool print_bytes, bool print_frames, bool print_errors);

bool serial_receive_take_frame(SerialReceiveFrame *out);
// 获取最后接收时间
/**
 * @brief 获取最后接收时间
 * 
 * @return uint32_t 最后接收时间
 */
uint32_t serial_receive_last_rx_ms();
// 获取接收统计信息
/**
 * @brief 获取接收统计信息
 * 
 * @return SerialReceiveStats 接收统计信息
 */ 
SerialReceiveStats serial_receive_stats();
