#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include "speed_pid.h"
#include "serial_sender.h"
#include "serial_receive.h"
#include <Ticker.h>
#include "imu.h"
#define PID_INTERVAL_US 50000

// 串口发送功能码：编码器总计数
static constexpr uint8_t FRAME_FUNC_ENCODER = 0x01;
static constexpr uint8_t FRAME_FUNC_IMU = 0x02;

static constexpr uint8_t CMD_FUNC_WHEEL_SPEED = 0x10;
static constexpr float CMD_SPEED_MAX_CM_S = 80.0f;
static constexpr uint32_t CMD_SPEED_TIMEOUT_MS = 500;
static constexpr int PWM_DEADBAND = 5;

static int16_t read_i16_le(const uint8_t *p);// 读取 16 位有符号整数，小端序

// 是否每秒打印一次调试信息
static constexpr bool DEBUG_SENSOR_PRINT = false;

// PID 控制器（左右轮各一套）
static PID pidL;
static PID pidR;

// 目标速度（cm/s），后续可通过串口指令修改
static float target_left_cm_s = 0.0f;
static float target_right_cm_s = 0.0f;
static uint32_t last_speed_cmd_ms = 0;
static bool speed_cmd_timed_out = false;

// 最近一次计算得到的状态（用于发送/打印）
static int32_t latest_left_count = 0;
static int32_t latest_right_count = 0;
static float latest_left_cm_s = 0.0f;
static float latest_right_cm_s = 0.0f;
static int latest_pwmL = 0;
static int latest_pwmR = 0;

extern float roll, pitch, yaw;
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

// 用 10ms tick 
static Ticker ticker;

// 由定时回调置位，在 loop() 里读取并清零
// 必须用 volatile，避免编译器优化导致读不到置位结果
static volatile bool due_50ms = false;
static volatile bool due_100ms = false;
static volatile bool due_1s = false;
static void on_tick_10ms();

void setup()
{
  Serial.begin(115200);

  // 底盘初始化
  wheel_encoder_init();
  wheel_encoder_speed_init();
  motor_init();
  while (!mpu6050_init()) {
  Serial.println("MPU6050 重试...");
  delay(500);
}
  mpu6050_calibrate();
  // PID 参数初始化
  pidL = {};
  pidR = {};
  PID_Init(&pidL, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);
  PID_Init(&pidR, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);

  target_left_cm_s = 0.0f;
  target_right_cm_s = 0.0f;

  // 单实例定时器：10ms tick
  ticker.attach_ms(10, on_tick_10ms);
}

void loop()
{
  const uint32_t now_ms = millis();
  serial_receive_update(Serial);
  SerialReceiveFrame rx;
  while (serial_receive_take_frame(&rx)) {
    if (rx.func == CMD_FUNC_WHEEL_SPEED && rx.payload_len >= 4) {
      const int16_t l = read_i16_le(&rx.payload[0]);
      const int16_t r = read_i16_le(&rx.payload[2]);
      target_left_cm_s = constrain(static_cast<float>(l) / 100.0f, -CMD_SPEED_MAX_CM_S, CMD_SPEED_MAX_CM_S);
      target_right_cm_s = constrain(static_cast<float>(r) / 100.0f, -CMD_SPEED_MAX_CM_S, CMD_SPEED_MAX_CM_S);
      last_speed_cmd_ms = now_ms;
      speed_cmd_timed_out = false;
    } 
  }

  if (last_speed_cmd_ms != 0 && (now_ms - last_speed_cmd_ms) > CMD_SPEED_TIMEOUT_MS) {
   if (!speed_cmd_timed_out) {
      // 停止电机 + 清空 PID
      target_left_cm_s = 0;
      target_right_cm_s = 0;
      PID_Clear(&pidL);
      PID_Clear(&pidR);
      motorA_set(0);
      motorB_set(0);
      latest_pwmL = 0;
      latest_pwmR = 0;
      speed_cmd_timed_out = true;
    }
  }

  // 2) 每 50ms 执行一次：PID
  if (due_50ms) {
    due_50ms = false;

    int32_t left_count = 0;
    int32_t right_count = 0;
    float left_cm_s = 0.0f;
    float right_cm_s = 0.0f;

    // 读取编码器：总计数 + 速度（cm/s）
    wheel_encoder_get_counts(&left_count, &right_count);
    const bool speed_updated = wheel_encoder_get_speed_cm_s(&left_cm_s, &right_cm_s, PID_INTERVAL_US);
    mpu6050_update();
    latest_left_count = left_count;
    latest_right_count = right_count;
    if (speed_updated) {
      latest_left_cm_s = left_cm_s;
      latest_right_cm_s = right_cm_s;
      if (fabs(target_left_cm_s) < 0.01) PID_Clear(&pidL);
      if (fabs(target_right_cm_s) < 0.01) PID_Clear(&pidR);
      float pwmL_float = PID_IncPIDCal(&pidL, left_cm_s, target_left_cm_s);
      float pwmR_float = PID_IncPIDCal(&pidR, right_cm_s, target_right_cm_s);
      int pwmL = constrain((int)pwmL_float, -255, 255);
      int pwmR = constrain((int)pwmR_float, -255, 255);
      if (abs(pwmL) < PWM_DEADBAND) pwmL = 0;
      if (abs(pwmR) < PWM_DEADBAND) pwmR = 0;
      motorA_set(pwmL);
      motorB_set(pwmR);

      latest_pwmL = pwmL;
      latest_pwmR = pwmR;
    }
  }
  // 3) 每 100ms 执行一次：发送传感器数据
  if (due_100ms && !DEBUG_SENSOR_PRINT) {
    due_100ms = false;
    uint8_t enc_payload[6];
    uint8_t imu_payload[18];
    ss_pack_int_le(latest_left_count, 3, &enc_payload[0]);//打包左轮总计数
    ss_pack_int_le(latest_right_count, 3, &enc_payload[3]);//打包右轮总计数
    ss_pack_mpu6050_bundle(ax,ay,az,gx,gy,gz,roll,pitch,yaw,imu_payload);//打包IMU数据帧
    ss_send(Serial, FRAME_FUNC_ENCODER, enc_payload, 6, false);//发送编码器总计数
    ss_send(Serial, FRAME_FUNC_IMU, imu_payload, 18, false);//发送IMU数据帧
  }



  // 3) 每 1 秒打印一次调试信息
  if (due_1s && DEBUG_SENSOR_PRINT) {
    due_1s = false;

    Serial.print("[ENC] countL=");
    Serial.print(latest_left_count);
    Serial.print(" countR=");
    Serial.print(latest_right_count);
    Serial.print(" vL_cm_s=");
    Serial.print(latest_left_cm_s);
    Serial.print(" vR_cm_s=");
    Serial.print(latest_right_cm_s);
    Serial.print(" pwmL=");
    Serial.print(latest_pwmL);
    Serial.print(" pwmR=");
    Serial.print(latest_pwmR);
    Serial.print(" roll=");
    Serial.print(roll);
    Serial.print(" pitch=");
    Serial.print(pitch);
    Serial.print(" yaw=");
    Serial.println(yaw);
  }
}

// 每 10ms 触发一次：用计数器合成 100ms 与 1s 的“到期标记”
static void on_tick_10ms()
{
  static uint16_t c50ms = 0;
  static uint16_t c100ms = 0;
  static uint16_t c1s = 0;

  c50ms++;
  c100ms++;
  c1s++;

  if (c50ms >= 5) {
    c50ms = 0;
    due_50ms = true;
  }

  if (c100ms >= 10) {
    c100ms = 0;
    due_100ms = true;
  }

  if (c1s >= 100) {
    c1s = 0;
    due_1s = true;
  }
}

static int16_t read_i16_le(const uint8_t *p)
{
  return static_cast<int16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

