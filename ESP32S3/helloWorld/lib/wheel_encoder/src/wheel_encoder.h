#pragma once

#include <Arduino.h>
#include <motor_driver.h>

// 默认编码器引脚（A相接中断，B相用于判断方向）
static constexpr uint8_t WHEEL_ENC_L_A_PIN = 6;// 左轮编码器A引脚
static constexpr uint8_t WHEEL_ENC_L_B_PIN = 7;// 左轮编码器B引脚
static constexpr uint8_t WHEEL_ENC_R_A_PIN = 42;// 右轮编码器A引脚
static constexpr uint8_t WHEEL_ENC_R_B_PIN = 41;// 右轮编码器B引脚

static constexpr int8_t WHEEL_ENC_L_SIGN = MOTOR_A_INVERT_DIR ? 1 : -1;// 左轮方向修正：1=不变，-1=取反
static constexpr int8_t WHEEL_ENC_R_SIGN = MOTOR_B_INVERT_DIR ? 1 : -1;// 右轮方向修正：1=不变，-1=取反

static constexpr float WHEEL_DIAMETER_CM = 7.0f;// 轮子直径（单位：厘米）
static constexpr float WHEEL_COUNTS_PER_REV = 600.0f; // 每转一次的计数
static constexpr float WHEEL_CM_PER_COUNT = (WHEEL_DIAMETER_CM * PI) / WHEEL_COUNTS_PER_REV; // 每个计数对应的厘米数
// 转换计数为厘米
inline float wheel_encoder_counts_to_distance_cm(int32_t counts)
{
  return static_cast<float>(counts) * WHEEL_CM_PER_COUNT;
}

// 计算速度（单位：厘米/秒）    
inline float wheel_encoder_delta_counts_to_speed_cm_s(int32_t delta_counts, float dt_s)
{
  return static_cast<float>(delta_counts) * WHEEL_CM_PER_COUNT / dt_s;
}

// 速度计算初始化（通常在 wheel_encoder_init 后调用一次）
void wheel_encoder_speed_init();

// 只计算左右轮速度，达到采样周期返回 true，否则返回 false
bool wheel_encoder_get_speed_cm_s(float *left_cm_s,
                                  float *right_cm_s,
                                  uint32_t sample_us = 100000U);

// 初始化编码器计数（enable_pullups=true 时使用内部上拉）
void wheel_encoder_init(bool enable_pullups = true);

// 只读取左右轮累计总计数（有符号，正负代表方向）
void wheel_encoder_get_counts(int32_t *left, int32_t *right);
int32_t wheel_encoder_get_left();
int32_t wheel_encoder_get_right();

// 清零计数/读取后清零（用于按周期计算增量里程）
void wheel_encoder_reset();
void wheel_encoder_get_and_reset(int32_t *left, int32_t *right);
