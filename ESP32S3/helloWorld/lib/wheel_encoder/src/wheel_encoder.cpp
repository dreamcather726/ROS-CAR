#include "wheel_encoder.h"

static volatile int32_t g_left = 0;// 左轮累计计数
static volatile int32_t g_right = 0;// 右轮累计计数
static volatile uint8_t g_left_state = 0;// 左轮当前状态
static volatile uint8_t g_right_state = 0;// 右轮当前状态
static int32_t g_speed_last_left = 0;// 上一次左轮速度计数
static int32_t g_speed_last_right = 0;// 上一次右轮速度计数
static uint32_t g_speed_last_us = 0;// 上一次速度采样时间（微秒）
static bool g_speed_inited = false;// 速度计算是否初始化
// 四相编码器状态转换表
static const int8_t g_quad_table[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};
// 更新左轮计数
static inline void IRAM_ATTR wheel_encoder_update_left()
{
  const uint8_t a = static_cast<uint8_t>(digitalRead(WHEEL_ENC_L_A_PIN));
  const uint8_t b = static_cast<uint8_t>(digitalRead(WHEEL_ENC_L_B_PIN));
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  const uint8_t table_idx = static_cast<uint8_t>((g_left_state << 2) | state);
  g_left += g_quad_table[table_idx];
  g_left_state = state;
}

static inline void IRAM_ATTR wheel_encoder_update_right()
{
  const uint8_t a = static_cast<uint8_t>(digitalRead(WHEEL_ENC_R_A_PIN));
  const uint8_t b = static_cast<uint8_t>(digitalRead(WHEEL_ENC_R_B_PIN));
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  const uint8_t table_idx = static_cast<uint8_t>((g_right_state << 2) | state);
  g_right += g_quad_table[table_idx];
  g_right_state = state;
}

static void IRAM_ATTR wheel_encoder_isr_left_a()
{
  wheel_encoder_update_left();
}

static void IRAM_ATTR wheel_encoder_isr_left_b()
{
  wheel_encoder_update_left();
}

static void IRAM_ATTR wheel_encoder_isr_right_a()
{
  wheel_encoder_update_right();
}

static void IRAM_ATTR wheel_encoder_isr_right_b()
{
  wheel_encoder_update_right();
}

void wheel_encoder_init(bool enable_pullups)
{
  pinMode(WHEEL_ENC_L_A_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_L_B_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_R_A_PIN, enable_pullups ? INPUT_PULLUP : INPUT);
  pinMode(WHEEL_ENC_R_B_PIN, enable_pullups ? INPUT_PULLUP : INPUT);

  g_left_state = static_cast<uint8_t>((digitalRead(WHEEL_ENC_L_A_PIN) << 1) | digitalRead(WHEEL_ENC_L_B_PIN));
  g_right_state = static_cast<uint8_t>((digitalRead(WHEEL_ENC_R_A_PIN) << 1) | digitalRead(WHEEL_ENC_R_B_PIN));

  detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_A_PIN));
  detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_B_PIN));
  detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_A_PIN));
  detachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_B_PIN));

  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_A_PIN), wheel_encoder_isr_left_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_L_B_PIN), wheel_encoder_isr_left_b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_A_PIN), wheel_encoder_isr_right_a, CHANGE);
  attachInterrupt(digitalPinToInterrupt(WHEEL_ENC_R_B_PIN), wheel_encoder_isr_right_b, CHANGE);
}

void wheel_encoder_speed_init()
{
  wheel_encoder_get_counts(&g_speed_last_left, &g_speed_last_right);
  g_speed_last_us = micros();
  g_speed_inited = true;
}

void wheel_encoder_get_speed_cm_s(float *left_cm_s, float *right_cm_s, uint32_t sample_us)
{
  if (!g_speed_inited) {
    wheel_encoder_speed_init();
    return;
  } 

  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - g_speed_last_us;
  if (dt_us == 0U) {
    return;
  }
  if (dt_us < sample_us) {
    return;
  }

  int32_t left = 0;
  int32_t right = 0;
  wheel_encoder_get_counts(&left, &right);

  const int32_t dl = left - g_speed_last_left;
  const int32_t dr = right - g_speed_last_right;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;
  const float v_left = wheel_encoder_delta_to_speed_cm_s(dl, dt_s);
  const float v_right = wheel_encoder_delta_to_speed_cm_s(dr, dt_s);

  if (left_cm_s) *left_cm_s = v_left;
  if (right_cm_s) *right_cm_s = v_right;

  g_speed_last_left = left;
  g_speed_last_right = right;
  g_speed_last_us = now_us;
  return;
}

bool wheel_encoder_get_odom(int32_t *left_count,
                            int32_t *right_count,
                            float *left_cm_s,
                            float *right_cm_s,
                            uint32_t sample_us)
{
  if (!g_speed_inited) {
    wheel_encoder_speed_init();
    return false;
  }

  const uint32_t now_us = micros();
  const uint32_t dt_us = now_us - g_speed_last_us;
  if (dt_us == 0U || dt_us < sample_us) {
    return false;
  }

  int32_t left = 0;
  int32_t right = 0;
  wheel_encoder_get_counts(&left, &right);

  const int32_t dl = left - g_speed_last_left;
  const int32_t dr = right - g_speed_last_right;
  const float dt_s = static_cast<float>(dt_us) * 1e-6f;
  const float v_left = wheel_encoder_delta_to_speed_cm_s(dl, dt_s);
  const float v_right = wheel_encoder_delta_to_speed_cm_s(dr, dt_s);

  if (left_count) *left_count = left;
  if (right_count) *right_count = right;
  if (left_cm_s) *left_cm_s = v_left;
  if (right_cm_s) *right_cm_s = v_right;

  g_speed_last_left = left;
  g_speed_last_right = right;
  g_speed_last_us = now_us;
  return true;
}

void wheel_encoder_get_counts(int32_t *left, int32_t *right)
{
  noInterrupts();
  const int32_t l = g_left;
  const int32_t r = g_right;
  interrupts();

  if (left) *left = l;
  if (right) *right = r;
}

int32_t wheel_encoder_get_left()
{
  noInterrupts();
  const int32_t l = g_left;
  interrupts();
  return l;
}

int32_t wheel_encoder_get_right()
{
  noInterrupts();
  const int32_t r = g_right;
  interrupts();
  return r;
}

void wheel_encoder_reset()
{
  noInterrupts();
  g_left = 0;
  g_right = 0;
  interrupts();
  wheel_encoder_speed_init();
}

void wheel_encoder_get_and_reset(int32_t *left, int32_t *right)
{
  noInterrupts();
  const int32_t l = g_left;
  const int32_t r = g_right;
  g_left = 0;
  g_right = 0;
  interrupts();

  if (left) *left = l;
  if (right) *right = r;
}
