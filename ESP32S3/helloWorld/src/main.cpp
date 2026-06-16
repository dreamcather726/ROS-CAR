#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include "speed_pid.h"
#include "serial_sender.h"
#include "serial_receive.h"
#include <Ticker.h>
#define ENABLE_MPU6050 1 // MPU6050 相关功能开关，0=关闭，1=开启。开启后会发送 IMU 数据帧，并在串口调试信息中打印姿态角。
#if ENABLE_MPU6050
#include "imu.h"
#endif
#include <math.h>
#include <string.h>
#define PID_INTERVAL_US 50000
#define ENABLE_ENCODER_TEST_COUNTER 0 // Encoder 测试计数器开关，1=开启，0=关闭。开启后会模拟编码器数据以测试 PID 控制和串口通信功能，无需实际连接编码器硬件。

// 下位机发送给上位机的功能码
static constexpr uint8_t FRAME_FUNC_ENCODER = 0x01;
#if ENABLE_MPU6050
static constexpr uint8_t FRAME_FUNC_IMU = 0x02;
#endif

// 上位机发送给下位机的功能码
static constexpr uint8_t CMD_FUNC_WHEEL_SPEED = 0x10;
static constexpr uint8_t CMD_FUNC_PID_CONTROL = 0x11;

static constexpr float CMD_SPEED_MAX_CM_S = 50.0f;
static constexpr uint32_t CMD_SPEED_TIMEOUT_MS = 500;
static constexpr int PWM_DEADBAND = 5;
static constexpr float DEFAULT_PID_KP = 2.5f;
static constexpr float DEFAULT_PID_KI =0.65f;
static constexpr float DEFAULT_PID_KD = 0.5f;
static constexpr float PID_OUTPUT_MAX = 255.0f;
static constexpr float PID_OUTPUT_MIN = -255.0f;
static constexpr float TARGET_SPEED_STEP_MAX_CM_S = 8.0f;
static constexpr int PWM_STEP_MAX = 12;
static constexpr int32_t TEST_ENCODER_COUNTS_PER_SEC = 20;
static constexpr uint32_t TEST_ENCODER_UPDATE_MS = 100;

static int16_t read_i16_le(const uint8_t *bytes);// 读取 16 位有符号整数，小端序
static float read_f32_le(const uint8_t *bytes);// 读取 32 位浮点数，小端序
static int16_t clamp_speed_cm_s_x100(float speed_cm_s);// 速度转换为 int16，单位 cm/s * 100
static int limit_pwm_step(int target_pwm, int current_pwm, int max_step);// 限制 PWM 单次变化量
static float limit_speed_step(float target_speed_cm_s, float current_speed_cm_s, float max_step_cm_s);// 限制目标速度单次变化量
static void apply_speed_command(float left_speed_cm_s, float right_speed_cm_s);// 应用目标速度命令
static void update_encoder_test_data(uint32_t now_ms);// 更新编码器测试数据
static void apply_pid_parameters(float kp, float ki, float kd);// 更新左右轮 PID 参数
static void PID_control(float left_speed_cm_s, float right_speed_cm_s);// 执行左右轮速度闭环

// 是否每秒打印一次调试信息
static constexpr bool DEBUG_SENSOR_PRINT = false;

// PID 控制器（左右轮各一套）
static PID left_speed_pid;
static PID right_speed_pid;

// 目标速度（cm/s），后续可通过串口指令修改
static float target_left_speed_cm_s = 0.0f;
static float target_right_speed_cm_s = 0.0f;
static float requested_left_speed_cm_s = 0.0f;
static float requested_right_speed_cm_s = 0.0f;
static uint32_t last_speed_command_ms = 0;
static bool has_speed_command_timed_out = false;

// 最近一次计算得到的状态（用于发送/打印）
static int32_t latest_left_encoder_count = 0;
static int32_t latest_right_encoder_count = 0;
static float latest_left_speed_cm_s = 0.0f;
static float latest_right_speed_cm_s = 0.0f;
static int latest_left_pwm = 0;
static int latest_right_pwm = 0;

#if ENABLE_MPU6050
extern float roll, pitch, yaw;
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;
#endif

// 用 10ms tick 
static Ticker control_ticker;

// 由定时回调置位，在 loop() 里读取并清零
// 必须用 volatile，避免编译器优化导致读不到置位结果
static volatile bool is_pid_update_due = false;
static volatile bool is_sensor_send_due = false;
static volatile bool is_debug_print_due = false;
static void on_tick_10ms();

void setup()
{
  Serial.begin(115200);

  // 底盘初始化
  wheel_encoder_init();
  wheel_encoder_speed_init();
  motor_init();
#if ENABLE_MPU6050
  while (!mpu6050_init()) {
  Serial.println("MPU6050 重试...");
  delay(500);
  }
  mpu6050_calibrate();
#endif
  // PID 参数初始化
  left_speed_pid = {};
  right_speed_pid = {};
  apply_pid_parameters(DEFAULT_PID_KP, DEFAULT_PID_KI, DEFAULT_PID_KD);

  target_left_speed_cm_s = 0.0f;
  target_right_speed_cm_s = 0.0f;

  // 单实例定时器：10ms tick
  control_ticker.attach_ms(10, on_tick_10ms);
}

void loop()
{
  const uint32_t now_ms = millis();

  // 1) 接收上位机命令：
  //    0x10 = 左右轮目标速度，单位 cm/s * 100，int16 小端。
  //    0x11 = PID_control 调参，依次为 Kp/Ki/Kd，float32 小端。
  serial_receive_update(Serial);
  SerialReceiveFrame received_frame;
  while (serial_receive_take_frame(&received_frame)) {
    if (received_frame.func == CMD_FUNC_WHEEL_SPEED && received_frame.payload_len >= 4) {
      const int16_t left_speed_raw_x100 = read_i16_le(&received_frame.payload[0]);
      const int16_t right_speed_raw_x100 = read_i16_le(&received_frame.payload[2]);
      apply_speed_command(static_cast<float>(left_speed_raw_x100) / 100.0f,
                          static_cast<float>(right_speed_raw_x100) / 100.0f);
      last_speed_command_ms = now_ms;
      has_speed_command_timed_out = false;
    } else if (received_frame.func == CMD_FUNC_PID_CONTROL && received_frame.payload_len >= 12) {
      const float kp = read_f32_le(&received_frame.payload[0]);
      const float ki = read_f32_le(&received_frame.payload[4]);
      const float kd = read_f32_le(&received_frame.payload[8]);
      apply_pid_parameters(kp, ki, kd);
    }
  }

  // 2) 如果上位机超过 500ms 没有继续发送速度命令，自动停车。
  if (last_speed_command_ms != 0 && (now_ms - last_speed_command_ms) > CMD_SPEED_TIMEOUT_MS) {
   if (!has_speed_command_timed_out) {
      // 停止电机 + 清空 PID
      requested_left_speed_cm_s = 0;
      requested_right_speed_cm_s = 0;
      target_left_speed_cm_s = 0;
      target_right_speed_cm_s = 0;
      PID_Clear(&left_speed_pid);
      PID_Clear(&right_speed_pid);
      set_right_motor_pwm(0);
      set_left_motor_pwm(0);
      latest_left_pwm = 0;
      latest_right_pwm = 0;
      has_speed_command_timed_out = true;
    }
  }

  // 3) 每 50ms 执行一次 PID 速度闭环。
  if (is_pid_update_due) {
    is_pid_update_due = false;

#if ENABLE_ENCODER_TEST_COUNTER
    update_encoder_test_data(now_ms);
#else
    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    float left_speed_cm_s = 0.0f;
    float right_speed_cm_s = 0.0f;

    // 读取编码器：总计数 + 速度（cm/s）
    wheel_encoder_get_counts(&left_encoder_count, &right_encoder_count);
    const bool is_speed_updated = wheel_encoder_get_speed_cm_s(&left_speed_cm_s, &right_speed_cm_s, PID_INTERVAL_US);
#if ENABLE_MPU6050
    mpu6050_update();
#endif
    latest_left_encoder_count = left_encoder_count;
    latest_right_encoder_count = right_encoder_count;
    if (is_speed_updated) {
      latest_left_speed_cm_s = left_speed_cm_s;
      latest_right_speed_cm_s = right_speed_cm_s;
      PID_control(left_speed_cm_s, right_speed_cm_s);
    }
#endif
  }

  // 4) 每 100ms 向上位机发送编码器速度，按需发送 IMU 数据。
  if (is_sensor_send_due && !DEBUG_SENSOR_PRINT) {
    is_sensor_send_due = false;
    uint8_t encoder_payload[10];
    ss_pack_int_le(latest_left_encoder_count, 3, &encoder_payload[0]);//打包左轮总计数
    ss_pack_int_le(latest_right_encoder_count, 3, &encoder_payload[3]);//打包右轮总计数
    ss_pack_int_le(clamp_speed_cm_s_x100(latest_left_speed_cm_s), 2, &encoder_payload[6]);//打包左轮实际速度
    ss_pack_int_le(clamp_speed_cm_s_x100(latest_right_speed_cm_s), 2, &encoder_payload[8]);//打包右轮实际速度
    ss_send(Serial, FRAME_FUNC_ENCODER, encoder_payload, 10, false);//发送编码器总计数和实际速度
#if ENABLE_MPU6050
    uint8_t imu_payload[18];
    ss_pack_mpu6050_bundle(ax,ay,az,gx,gy,gz,roll,pitch,yaw,imu_payload);//打包IMU数据帧
    ss_send(Serial, FRAME_FUNC_IMU, imu_payload, 18, false);//发送IMU数据帧
#endif
  }



  // 5) 如果打开 DEBUG_SENSOR_PRINT，每 1 秒打印一次调试信息。
  if (is_debug_print_due && DEBUG_SENSOR_PRINT) {
    is_debug_print_due = false;

    Serial.print("[ENC] countL=");
    Serial.print(latest_left_encoder_count);
    Serial.print(" countR=");
    Serial.print(latest_right_encoder_count);
    Serial.print(" vL_cm_s=");
    Serial.print(latest_left_speed_cm_s);
    Serial.print(" vR_cm_s=");
    Serial.print(latest_right_speed_cm_s);
    Serial.print(" pwmL=");
    Serial.print(latest_left_pwm);
    Serial.print(" pwmR=");
    Serial.print(latest_right_pwm);
#if ENABLE_MPU6050
    Serial.print(" roll=");
    Serial.print(roll);
    Serial.print(" pitch=");
    Serial.print(pitch);
    Serial.print(" yaw=");
    Serial.println(yaw);
#else
    Serial.println();
#endif
  }
}

// 每 10ms 触发一次：用计数器合成 100ms 与 1s 的“到期标记”
static void on_tick_10ms()
{
  static uint16_t pid_tick_count = 0;
  static uint16_t sensor_send_tick_count = 0;
  static uint16_t debug_print_tick_count = 0;

  pid_tick_count++;
  sensor_send_tick_count++;
  debug_print_tick_count++;

  if (pid_tick_count >= 5) {
    pid_tick_count = 0;
    is_pid_update_due = true;
  }

  if (sensor_send_tick_count >= 10) {
    sensor_send_tick_count = 0;
    is_sensor_send_due = true;
  }

  if (debug_print_tick_count >= 100) {
    debug_print_tick_count = 0;
    is_debug_print_due = true;
  }
}

static int16_t read_i16_le(const uint8_t *bytes)
{
  return static_cast<int16_t>(static_cast<uint16_t>(bytes[0]) | (static_cast<uint16_t>(bytes[1]) << 8));
}

static float read_f32_le(const uint8_t *bytes)
{
  const uint32_t raw = static_cast<uint32_t>(bytes[0])
                     | (static_cast<uint32_t>(bytes[1]) << 8)
                     | (static_cast<uint32_t>(bytes[2]) << 16)
                     | (static_cast<uint32_t>(bytes[3]) << 24);
  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  return value;
}

static int16_t clamp_speed_cm_s_x100(float speed_cm_s)
{
  const int32_t scaled_speed = static_cast<int32_t>(lroundf(speed_cm_s * 100.0f));
  if (scaled_speed > 32767) return 32767;
  if (scaled_speed < -32768) return -32768;
  return static_cast<int16_t>(scaled_speed);
}

static int limit_pwm_step(int target_pwm, int current_pwm, int max_step)
{
  const int delta_pwm = target_pwm - current_pwm;
  if (delta_pwm > max_step) return current_pwm + max_step;
  if (delta_pwm < -max_step) return current_pwm - max_step;
  return target_pwm;
}

static float limit_speed_step(float target_speed_cm_s, float current_speed_cm_s, float max_step_cm_s)
{
  const float delta_speed_cm_s = target_speed_cm_s - current_speed_cm_s;
  if (delta_speed_cm_s > max_step_cm_s) return current_speed_cm_s + max_step_cm_s;
  if (delta_speed_cm_s < -max_step_cm_s) return current_speed_cm_s - max_step_cm_s;
  return target_speed_cm_s;
}

static void apply_speed_command(float left_speed_cm_s, float right_speed_cm_s)
{
  const float new_left_speed_cm_s = constrain(left_speed_cm_s, -CMD_SPEED_MAX_CM_S, CMD_SPEED_MAX_CM_S);
  const float new_right_speed_cm_s = constrain(right_speed_cm_s, -CMD_SPEED_MAX_CM_S, CMD_SPEED_MAX_CM_S);

  const bool is_left_direction_changed = (requested_left_speed_cm_s * new_left_speed_cm_s) < 0.0f;
  const bool is_right_direction_changed = (requested_right_speed_cm_s * new_right_speed_cm_s) < 0.0f;
  if (is_left_direction_changed || (fabs(new_left_speed_cm_s) < 0.01f && fabs(requested_left_speed_cm_s) >= 0.01f)) {
    PID_Clear(&left_speed_pid);
  }
  if (is_right_direction_changed || (fabs(new_right_speed_cm_s) < 0.01f && fabs(requested_right_speed_cm_s) >= 0.01f)) {
    PID_Clear(&right_speed_pid);
  }

  requested_left_speed_cm_s = new_left_speed_cm_s;
  requested_right_speed_cm_s = new_right_speed_cm_s;
}

static void update_encoder_test_data(uint32_t now_ms)
{
  static uint32_t last_test_update_ms = 0;
  if (last_test_update_ms == 0) {
    last_test_update_ms = now_ms;
    return;
  }

  const uint32_t elapsed_ms = now_ms - last_test_update_ms;
  if (elapsed_ms < TEST_ENCODER_UPDATE_MS) {
    return;
  }

  const int32_t step_count = static_cast<int32_t>(
      (static_cast<uint32_t>(TEST_ENCODER_COUNTS_PER_SEC) * elapsed_ms)
      / 1000U);
  if (step_count <= 0) {
    return;
  }

  latest_left_encoder_count += step_count;
  latest_right_encoder_count += step_count;
  latest_left_speed_cm_s = TEST_ENCODER_COUNTS_PER_SEC * WHEEL_CM_PER_COUNT;
  latest_right_speed_cm_s = TEST_ENCODER_COUNTS_PER_SEC * WHEEL_CM_PER_COUNT;
  last_test_update_ms = now_ms;
}

static void apply_pid_parameters(float kp, float ki, float kd)
{
  if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
    return;
  }

  PID_Init(&left_speed_pid, kp, ki, kd, PID_OUTPUT_MAX, PID_OUTPUT_MIN);
  PID_Init(&right_speed_pid, kp, ki, kd, PID_OUTPUT_MAX, PID_OUTPUT_MIN);
  PID_Clear(&left_speed_pid);
  PID_Clear(&right_speed_pid);
}

static void PID_control(float left_speed_cm_s, float right_speed_cm_s)
{
  target_left_speed_cm_s = limit_speed_step(requested_left_speed_cm_s,
                                           target_left_speed_cm_s,
                                           TARGET_SPEED_STEP_MAX_CM_S);
  target_right_speed_cm_s = limit_speed_step(requested_right_speed_cm_s,
                                            target_right_speed_cm_s,
                                            TARGET_SPEED_STEP_MAX_CM_S);

  if (fabs(target_left_speed_cm_s) < 0.01f) PID_Clear(&left_speed_pid);
  if (fabs(target_right_speed_cm_s) < 0.01f) PID_Clear(&right_speed_pid);

  const float left_pwm_float = PID_IncPIDCal(&left_speed_pid, left_speed_cm_s, target_left_speed_cm_s);
  const float right_pwm_float = PID_IncPIDCal(&right_speed_pid, right_speed_cm_s, target_right_speed_cm_s);
  int left_pwm = constrain(static_cast<int>(left_pwm_float), -255, 255);
  int right_pwm = constrain(static_cast<int>(right_pwm_float), -255, 255);

  if (abs(left_pwm) < PWM_DEADBAND) left_pwm = 0;
  if (abs(right_pwm) < PWM_DEADBAND) right_pwm = 0;
  left_pwm = limit_pwm_step(left_pwm, latest_left_pwm, PWM_STEP_MAX);
  right_pwm = limit_pwm_step(right_pwm, latest_right_pwm, PWM_STEP_MAX);

  set_left_motor_pwm(left_pwm);
  set_right_motor_pwm(right_pwm);

  latest_left_pwm = left_pwm;
  latest_right_pwm = right_pwm;
}
