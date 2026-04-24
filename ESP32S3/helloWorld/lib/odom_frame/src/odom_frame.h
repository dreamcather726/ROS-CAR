#pragma once

#include <Arduino.h>

// 构建6字节的odom_frame，包含24位有符号整数（小端）
bool odom_frame_build_counts_i24_le(int32_t left_count, int32_t right_count, uint8_t out6[6]);
// 构建6字节的odom_frame，包含16位有符号整数（小端），速度值乘以100
bool odom_frame_build_speed_i16_le_x100(float left_cm_s, float right_cm_s, uint8_t out6[6]);    
