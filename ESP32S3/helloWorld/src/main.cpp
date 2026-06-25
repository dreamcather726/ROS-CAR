#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <arm_servo.h>
#include <arm_wifi_control.h>
#include "speed_pid.h"
#include "serial_sender.h"
#include "serial_receive.h"
#include <Ticker.h>
#include "imu.h"
#include <math.h>
#define PID_INTERVAL_US 50000

// 串口发送功能码：编码器总计数
static constexpr uint8_t FRAME_FUNC_ENCODER = 0x01;
static constexpr uint8_t FRAME_FUNC_IMU = 0x02;

static constexpr uint8_t CMD_FUNC_WHEEL_SPEED = 0x10;
static constexpr float CMD_SPEED_MAX_CM_S = 80.0f;
static constexpr uint32_t CMD_SPEED_TIMEOUT_MS = 500;
static constexpr int PWM_DEADBAND = 5;
static constexpr char ARM_WIFI_STA_SSID[] = "具身智能机器人-2.4G";
static constexpr char ARM_WIFI_STA_PASSWORD[] = "jushen123";

static int16_t read_i16_le(const uint8_t *bytes);// 读取 16 位有符号整数，小端序
static int16_t clamp_speed_cm_s_x100(float speed_cm_s);// 速度转换为 int16，单位 cm/s * 100

// 是否每秒打印一次调试信息
static constexpr bool DEBUG_SENSOR_PRINT = false;

static ArmServo armServo;
static ArmWifiControl armWifiControl;

// PID 控制器（左右轮各一套）
static PID left_speed_pid;
static PID right_speed_pid;

// 目标速度（cm/s），后续可通过串口指令修改
static float target_left_speed_cm_s = 0.0f;
static float target_right_speed_cm_s = 0.0f;
static uint32_t last_speed_command_ms = 0;
static bool has_speed_command_timed_out = false;

// 最近一次计算得到的状态（用于发送/打印）
static int32_t latest_left_encoder_count = 0;
static int32_t latest_right_encoder_count = 0;
static float latest_left_speed_cm_s = 0.0f;
static float latest_right_speed_cm_s = 0.0f;
static int latest_left_pwm = 0;
static int latest_right_pwm = 0;

extern float roll, pitch, yaw;
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

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

  armServo.begin();
  armWifiControl.begin(armServo, ARM_WIFI_STA_SSID, ARM_WIFI_STA_PASSWORD);

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
  left_speed_pid = {};
  right_speed_pid = {};
  PID_Init(&left_speed_pid, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);
  PID_Init(&right_speed_pid, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);

  target_left_speed_cm_s = 0.0f;
  target_right_speed_cm_s = 0.0f;

  // 单实例定时器：10ms tick
  control_ticker.attach_ms(10, on_tick_10ms);
}

void loop()
{
  const uint32_t now_ms = millis();
  armServo.update();
  armWifiControl.update();

  if (armWifiControl.isSpeedLimited()) {
    armWifiControl.limitTargetSpeeds(&target_left_speed_cm_s, &target_right_speed_cm_s);
  }

  serial_receive_update(Serial);
  SerialReceiveFrame received_frame;
  while (serial_receive_take_frame(&received_frame)) {
    if (received_frame.func == CMD_FUNC_WHEEL_SPEED && received_frame.payload_len >= 4) {
      const int16_t left_speed_raw_x100 = read_i16_le(&received_frame.payload[0]);
      const int16_t right_speed_raw_x100 = read_i16_le(&received_frame.payload[2]);
      const float speed_limit_cm_s = armWifiControl.isSpeedLimited() ? armWifiControl.getSpeedLimitCmS() : CMD_SPEED_MAX_CM_S;
      target_left_speed_cm_s = constrain(static_cast<float>(left_speed_raw_x100) / 100.0f,
                                         -speed_limit_cm_s,
                                         speed_limit_cm_s);
      target_right_speed_cm_s = constrain(static_cast<float>(right_speed_raw_x100) / 100.0f,
                                          -speed_limit_cm_s,
                                          speed_limit_cm_s);
      last_speed_command_ms = now_ms;
      has_speed_command_timed_out = false;
    } 
  }

  if (last_speed_command_ms != 0 && (now_ms - last_speed_command_ms) > CMD_SPEED_TIMEOUT_MS) {
   if (!has_speed_command_timed_out) {
      // 停止电机 + 清空 PID
      target_left_speed_cm_s = 0;
      target_right_speed_cm_s = 0;
      PID_Clear(&left_speed_pid);
      PID_Clear(&right_speed_pid);
      motorA_set(0);
      motorB_set(0);
      latest_left_pwm = 0;
      latest_right_pwm = 0;
      has_speed_command_timed_out = true;
    }
  }

  // 2) 每 50ms 执行一次：PID
  if (is_pid_update_due) {
    is_pid_update_due = false;

    int32_t left_encoder_count = 0;
    int32_t right_encoder_count = 0;
    float left_speed_cm_s = 0.0f;
    float right_speed_cm_s = 0.0f;

    // 读取编码器：总计数 + 速度（cm/s）
    wheel_encoder_get_counts(&left_encoder_count, &right_encoder_count);
    const bool is_speed_updated = wheel_encoder_get_speed_cm_s(&left_speed_cm_s, &right_speed_cm_s, PID_INTERVAL_US);
    mpu6050_update();
    latest_left_encoder_count = left_encoder_count;
    latest_right_encoder_count = right_encoder_count;
    if (is_speed_updated) {
      if (armWifiControl.isSpeedLimited()) {
        armWifiControl.limitTargetSpeeds(&target_left_speed_cm_s, &target_right_speed_cm_s);
      }

      latest_left_speed_cm_s = left_speed_cm_s;
      latest_right_speed_cm_s = right_speed_cm_s;
      if (fabs(target_left_speed_cm_s) < 0.01) PID_Clear(&left_speed_pid);
      if (fabs(target_right_speed_cm_s) < 0.01) PID_Clear(&right_speed_pid);
      float left_pwm_float = PID_IncPIDCal(&left_speed_pid, left_speed_cm_s, target_left_speed_cm_s);
      float right_pwm_float = PID_IncPIDCal(&right_speed_pid, right_speed_cm_s, target_right_speed_cm_s);
      int left_pwm = constrain((int)left_pwm_float, -255, 255);
      int right_pwm = constrain((int)right_pwm_float, -255, 255);
      if (abs(left_pwm) < PWM_DEADBAND) left_pwm = 0;
      if (abs(right_pwm) < PWM_DEADBAND) right_pwm = 0;
      motorA_set(left_pwm);
      motorB_set(right_pwm);

      latest_left_pwm = left_pwm;
      latest_right_pwm = right_pwm;
    }
  }
  // 3) 每 100ms 执行一次：发送传感器数据
  if (is_sensor_send_due && !DEBUG_SENSOR_PRINT) {
    is_sensor_send_due = false;
    uint8_t encoder_payload[10];
    uint8_t imu_payload[18];
    ss_pack_int_le(latest_left_encoder_count, 3, &encoder_payload[0]);//打包左轮总计数
    ss_pack_int_le(latest_right_encoder_count, 3, &encoder_payload[3]);//打包右轮总计数
    ss_pack_int_le(clamp_speed_cm_s_x100(latest_left_speed_cm_s), 2, &encoder_payload[6]);//打包左轮实际速度
    ss_pack_int_le(clamp_speed_cm_s_x100(latest_right_speed_cm_s), 2, &encoder_payload[8]);//打包右轮实际速度
    ss_pack_mpu6050_bundle(ax,ay,az,gx,gy,gz,roll,pitch,yaw,imu_payload);//打包IMU数据帧
    ss_send(Serial, FRAME_FUNC_ENCODER, encoder_payload, 10, false);//发送编码器总计数和实际速度
    ss_send(Serial, FRAME_FUNC_IMU, imu_payload, 18, false);//发送IMU数据帧
  }



  // 3) 每 1 秒打印一次调试信息
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

static int16_t clamp_speed_cm_s_x100(float speed_cm_s)
{
  const int32_t scaled_speed = static_cast<int32_t>(lroundf(speed_cm_s * 100.0f));
  if (scaled_speed > 32767) return 32767;
  if (scaled_speed < -32768) return -32768;
  return static_cast<int16_t>(scaled_speed);
}

