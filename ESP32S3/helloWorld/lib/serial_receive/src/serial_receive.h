#pragma once

#include <Arduino.h>

static constexpr uint8_t SERIAL_RECEIVE_HEADER = 0xAA;// 0xAA: 头部
static constexpr uint8_t SERIAL_RECEIVE_TAIL = 0xBB;// 0xBB: 尾部
static constexpr size_t SERIAL_RECEIVE_FRAME_SIZE = 10;// 10: 帧大小
static constexpr size_t SERIAL_RECEIVE_DATA_SIZE = 6;// 6: 数据大小

static constexpr uint8_t SERIAL_RECEIVE_FUNC_WHEEL_SPEED = 0x10;// 0x10: 轮子速度
static constexpr uint8_t SERIAL_RECEIVE_FUNC_WHEEL_TARGET_COUNTS = 0x11;// 0x11: 目标计数
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
// 提取轮子速度
/**
 * @brief 提取轮子速度
 * 
 * @param left_cm_s 左轮子速度指针
 * @param right_cm_s 右轮子速度指针
 * @param rx_ms 接收时间指针
 * @return true 成功
 * @return false 失败
 */
bool serial_receive_take_wheel_speed_cm_s(float *left_cm_s,
                                         float *right_cm_s,
                                         uint32_t *rx_ms = nullptr);
/**
 * @brief 提取目标计数
 * 
 * @param left_counts 左目标计数指针
 * @param right_counts 右目标计数指针
 * @param rx_ms 接收时间指针
 * @return true 成功
 * @return false 失败
 */
/**
 * @brief 提取目标计数
 * 
 * @param left_counts 左目标计数指针
 * @param right_counts 右目标计数指针
 * @param rx_ms 接收时间指针
 * @return true 成功
 * @return false 失败
 */
bool serial_receive_take_wheel_target_counts(int32_t *left_counts,
                                             int32_t *right_counts,
                                             uint32_t *rx_ms = nullptr);
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
