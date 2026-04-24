#pragma once

#include <Arduino.h>

// 默认编码器引脚（A相接中断，B相用于判断方向）
static constexpr uint8_t WHEEL_ENC_L_A_PIN = 17;// 左轮编码器A引脚
static constexpr uint8_t WHEEL_ENC_L_B_PIN = 18;// 左轮编码器B引脚
static constexpr uint8_t WHEEL_ENC_R_A_PIN = 3;// 右轮编码器A引脚
static constexpr uint8_t WHEEL_ENC_R_B_PIN = 46;// 右轮编码器B引脚

static constexpr float WHEEL_ODOM_DIAMETER_CM = 6.5f;// 轮子直径（单位：厘米）
static constexpr float WHEEL_ODOM_COUNTS_PER_REV = 1335.0f; // 每转一次的计数
static constexpr float WHEEL_ODOM_CM_PER_COUNT = (WHEEL_ODOM_DIAMETER_CM * PI) / WHEEL_ODOM_COUNTS_PER_REV; // 每个计数对应的厘米数
// 转换计数为厘米
inline float wheel_encoder_counts_to_cm(int32_t counts)
{
  return static_cast<float>(counts) * WHEEL_ODOM_CM_PER_COUNT;
}

// 计算速度（单位：厘米/秒）    
inline float wheel_encoder_delta_to_speed_cm_s(int32_t delta_counts, float dt_s)
{
  return static_cast<float>(delta_counts) * WHEEL_ODOM_CM_PER_COUNT / dt_s;
}

// 速度计算初始化（通常在 wheel_encoder_init 后调用一次）
void wheel_encoder_speed_init();

// 按固定采样周期计算速度，达到周期返回 true，否则返回 false
void wheel_encoder_get_speed_cm_s(float *left_cm_s, float *right_cm_s, uint32_t sample_us = 100000U);

bool wheel_encoder_get_odom(int32_t *left_count,
                            int32_t *right_count,
                            float *left_cm_s,
                            float *right_cm_s,
                            uint32_t sample_us = 100000U);

// 初始化编码器计数（enable_pullups=true 时使用内部上拉）
void wheel_encoder_init(bool enable_pullups = true);

// 读取左右轮累计计数（有符号，正负代表方向）
void wheel_encoder_get_counts(int32_t *left, int32_t *right);
int32_t wheel_encoder_get_left();
int32_t wheel_encoder_get_right();

// 清零计数/读取后清零（用于按周期计算增量里程）
void wheel_encoder_reset();
void wheel_encoder_get_and_reset(int32_t *left, int32_t *right);
