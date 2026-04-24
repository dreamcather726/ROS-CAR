#pragma once

#include <Arduino.h>

void odom_frame_build_counts_i24_le(int32_t left_count, int32_t right_count, uint8_t out6[6]);
void odom_frame_build_speed_i16_le_x100(float left_cm_s, float right_cm_s, uint8_t out6[6]);
