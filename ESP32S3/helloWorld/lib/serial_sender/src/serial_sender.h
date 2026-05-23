#pragma once

#include <Arduino.h>

// 新数据帧格式：
// 帧头(1字节) + 长度(1字节) + 数据区(可变长度) + CRC16(2字节) + 帧尾(1字节)
//
// 其中“数据区”的格式为：
// func(1字节) + payload(0~N字节)
//
// 长度字段的值 = 数据区总长度 = 1(func) + payload长度
static constexpr uint8_t FRAME_HEADER = 0xAA;// 帧头
static constexpr uint8_t FRAME_TAIL = 0xBB;// 帧尾
static constexpr size_t SERIAL_SENDER_CRC_SIZE = 2;          // CRC16 占 2 字节
static constexpr size_t SERIAL_SENDER_MAX_PAYLOAD_SIZE = 32; // 允许的最大业务数据长度
static constexpr size_t SERIAL_SENDER_MIN_DATA_SIZE = 1;     // 数据区最小只有 1 个 func
static constexpr size_t SERIAL_SENDER_MAX_DATA_SIZE = 1 + SERIAL_SENDER_MAX_PAYLOAD_SIZE;
static constexpr size_t SERIAL_SENDER_MIN_FRAME_SIZE = 6;    // AA LEN FUNC CRC_L CRC_H BB
static constexpr size_t SERIAL_SENDER_MAX_FRAME_SIZE = 1 + 1 + SERIAL_SENDER_MAX_DATA_SIZE + SERIAL_SENDER_CRC_SIZE + 1;// 最大帧长度

// 计算 CRC16（Modbus 常用多项式 0xA001）
uint16_t ss_crc16(const uint8_t bytes[], size_t len);

// 把一个整数按“小端格式”打包到数组中。
// 例如：
// byte_count = 2 时，可打包 int16
// byte_count = 3 时，可打包编码器 24 位计数
void ss_pack_int_le(int32_t value,
                    uint8_t byte_count,
                    uint8_t out[]);

// 直接发送左右编码器计数。
// 这个函数内部固定使用 6 字节 payload，
// 主程序就不用自己再声明 data_counts[6] 了。
void ss_send_encoder_counts(HardwareSerial &serial,
                            uint8_t func,
                            int32_t left_count,
                            int32_t right_count,
                            bool flush_after_send = true);

// 直接发送超声波距离。
// 发送格式：距离(cm) * 10 后转成 int16，再走 CRC 帧发送。
// 如果距离无效（小于 0），发送值为 -1。
void ss_send_ultrasonic_distance_cm_x10(HardwareSerial &serial,
                                        uint8_t func,
                                        float distance_cm,
                                        bool flush_after_send = true);

// 构建一帧。
// 返回值：实际组好的帧长度；如果返回 0，说明 payload_len 超出限制。
//
// out 的格式为：
// AA | LEN | FUNC | payload... | CRC_L | CRC_H | BB
uint8_t ss_build_frame(uint8_t func,// 功能码
                       const uint8_t payload[],// 数据区
                       uint8_t payload_len,// 数据区长度
                       uint8_t out[SERIAL_SENDER_MAX_FRAME_SIZE]);

// 发送一帧
void ss_send(HardwareSerial &serial,
             uint8_t func,
             const uint8_t payload[],
             uint8_t payload_len,
             bool flush_after_send = true // 是否发送后刷新串口缓冲区
             );
