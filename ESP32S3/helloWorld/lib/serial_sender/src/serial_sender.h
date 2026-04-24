#pragma once

#include <Arduino.h>

// 10字节固定帧：AA | func | d0 d1 d2 d3 d4 d5 | checksum | BB
static constexpr uint8_t SERIAL_SENDER_HEADER = 0xAA;
static constexpr uint8_t SERIAL_SENDER_TAIL = 0xBB;
static constexpr size_t SERIAL_SENDER_FRAME_SIZE = 10;
static constexpr size_t SERIAL_SENDER_DATA_SIZE = 6;

// 计算校验和：对 frame[0..len-1] 逐字节求和，返回低8位
uint8_t ss_checksum8(const uint8_t *bytes, size_t len);

// 构建一帧（out长度必须为SERIAL_SENDER_FRAME_SIZE；data为6字节，可传nullptr表示全0）
void ss_build_frame(uint8_t func,
                    const uint8_t data[SERIAL_SENDER_DATA_SIZE],
                    uint8_t out[SERIAL_SENDER_FRAME_SIZE]);

// 发送一帧（data为6字节，可传nullptr表示全0；flush_after_send=true会flush保证发送完成）
void ss_send(HardwareSerial &serial,
             uint8_t func,
             const uint8_t data[SERIAL_SENDER_DATA_SIZE] = nullptr,
             bool flush_after_send = true);

// 发送一帧（直接传6个数据字节）
void ss_send6(HardwareSerial &serial,
              uint8_t func,
              uint8_t d0,
              uint8_t d1,
              uint8_t d2,
              uint8_t d3,
              uint8_t d4,
              uint8_t d5,
              bool flush_after_send = true);
