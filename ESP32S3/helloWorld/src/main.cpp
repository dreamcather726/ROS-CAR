
#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <serial_sender.h>
#include <ultrasonic.h>
#include <odom_frame.h>
#include <mpu6050.h>
#include "speed_pid.h"
#include <odom_timer.h>
#include <serial_receive.h> 
#define SERIAL_BAUD 115200

static constexpr uint8_t FUNC_ODOM_COUNTS = 0x01;// 0x01: 速度计数
static constexpr uint8_t FUNC_ODOM_SPEED = 0x02;// 0x02: 速度
static constexpr uint8_t FUNC_ODOM_DISTANCE = 0x03;// 0x03: 距离
static constexpr uint8_t FUNC_IMU_ROLL_PITCH_YAW = 0x04;//角度
static constexpr bool SEND_TO_MUSEPI = true;// 发送到 MUSEPI
static constexpr bool DEBUG_PID_PRINT = false;// 打印 PID 值
static constexpr bool DEBUG_SERIAL_RX = true;// 打印串口接收数据

// PID 周期 100ms（必须固定！）
static constexpr uint32_t PID_PERIOD_US = 100000U;// 100ms 周期
static Timer pid_timer = {1};
static constexpr uint32_t SENSOR_READ_PERIOD_US = 100000U;// 100ms 周期
static Timer sensor_read_timer = {2};
static constexpr uint32_t TX_PERIOD_US = 20000U;// 20ms 周期
static Timer tx_timer = {0};

// 只使用左轮 PID
static PID pidL;
static PID pidR;
float target_cm_s = 0.0f;
static float target_left_cm_s = 0.0f;
static float target_right_cm_s = 0.0f;
float left_cm_s = 0.0f;
float right_cm_s = 0.0f;
float pwmL_float = 0.0f;
float pwmR_float = 0.0f;
int pwmL = 0;
int pwmR = 0;
static int32_t cmd_target_left_counts = 0;
static int32_t cmd_target_right_counts = 0;
static int32_t g_left_count = 0;// 左轮速度计数器
static int32_t g_right_count = 0;// 右轮速度计数器
static float g_left_cm_s = 0.0f;// 左轮速度
static float g_right_cm_s = 0.0f;// 右轮速度
static float g_ultrasonic_cm = -1.0f;// 超声波距离
static float g_roll_deg = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_yaw_deg = 0.0f;
void setup() {
  Serial.begin(SERIAL_BAUD);
  if (DEBUG_SERIAL_RX) {
    serial_receive_set_debug(&Serial);
    serial_receive_set_debug_flags(true, true, true);
  } 
  ultrasonic_init();// 初始化超声波传感器
  wheel_encoder_init();// 初始化轮速编码器
  mpu6050_init();// 初始化 MPUU
  mpu6050_calibrate();// 校准 MPUU
  wheel_encoder_speed_init();// 初始化轮速编码器速度
  motor_init();// 初始化电机
  timer_init(&pid_timer, 1, PID_PERIOD_US);// 初始化 PID 周期
  timer_init(&sensor_read_timer, 2, SENSOR_READ_PERIOD_US);// 初始化传感器读取周期
  timer_init(&tx_timer, 0, TX_PERIOD_US);// 初始化发送周期
  // PID_Init(&pidL,0.5f, 1.4f, 0.2f, 255.0f, -255.0f);
  pidL = {};// 初始化 PID 结构体
  pidR = {};// 初始化 PID 结构体
  PID_Init(&pidL, pidL.P+1.35f, pidL.I+1.2f, pidL.D+0.02f, 255.0f, -255.0f);// 初始化 PID
  PID_Init(&pidR, pidR.P+1.35f, pidR.I+1.2f, pidR.D+0.02f, 255.0f, -255.0f);// 初始化 PID
}
void loop() {
  serial_receive_update(Serial);
  float rx_target_left_cm_s = 1.0f;
  float rx_target_right_cm_s =  1.0f;
  if (serial_receive_take_wheel_speed_cm_s(&rx_target_left_cm_s, &rx_target_right_cm_s, nullptr)) {
    target_left_cm_s = rx_target_left_cm_s;
    target_right_cm_s = rx_target_right_cm_s;
    target_cm_s = (target_left_cm_s + target_right_cm_s) * 0.5f;
  }

  int32_t rx_target_left_counts = 0;
  int32_t rx_target_right_counts = 0;
  if (serial_receive_take_wheel_target_counts(&rx_target_left_counts, &rx_target_right_counts, nullptr)) {
    cmd_target_left_counts = rx_target_left_counts;
    cmd_target_right_counts = rx_target_right_counts;
  }


  if (timer_due(&pid_timer)) {// PID 周期
    wheel_encoder_get_odom(&g_left_count, &g_right_count, &g_left_cm_s, &g_right_cm_s, 0U);// 获取轮速编码器速度
    left_cm_s = g_left_cm_s;
    right_cm_s = g_right_cm_s;
    pwmL_float = PID_IncPIDCal(&pidL, left_cm_s, target_left_cm_s);// 计算左轮 PWM 值
    pwmR_float = PID_IncPIDCal(&pidR, right_cm_s, target_right_cm_s);// 计算右轮 PWM 值
    pwmL = constrain((int)pwmL_float, -255, 255); 
    pwmR = constrain((int)pwmR_float, -255, 255);// 修正 pwmR 的约束范围，从 -2555 改为 -255 保持一致性
    motorA_set(pwmL);// 设置左轮 PWM 值
    motorB_set(pwmR);// 设置右轮 PWM 值
    
    if (DEBUG_PID_PRINT && !SEND_TO_MUSEPI) {
      Serial.print(target_cm_s);
      Serial.print(",");
      Serial.println(left_cm_s);
      // Serial.print(",");
      // Serial.println(pwmL);
      // Serial.print(",");
      // Serial.print(right_cm_s);
      // Serial.print(",");
      // Serial.println(pwmR);
    }
  }

  if (timer_due(&sensor_read_timer)) {// 传感器读取周期
    g_ultrasonic_cm = ultrasonic_get_distance();// 获取超声波传感器距离
    mpu6050_update();// 更新 MPUU 数据
    g_roll_deg = roll;
    g_pitch_deg = pitch;
    g_yaw_deg = yaw;
    }

    

  if (SEND_TO_MUSEPI && timer_due(&tx_timer)) {// 发送数据
    if (Serial.availableForWrite() < static_cast<int>(SERIAL_SENDER_FRAME_SIZE * 4)) {
      return;
    }

    uint8_t data_counts[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_speed[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_distance[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_roll_pitch_yaw[SERIAL_SENDER_DATA_SIZE];

    odom_frame_build_counts_i24_le(g_left_count, g_right_count, data_counts);// 构建轮速编码器数据
    odom_frame_build_speed_i16_le_x100(g_left_cm_s, g_right_cm_s, data_speed);// 构建轮速编码器速度数据
    ultrasonic_pack_distance_cm_x10(g_ultrasonic_cm, data_distance);// 构建超声波传感器距离数据
    mpu6050_pack_roll_pitch_yaw_degrees(g_roll_deg, g_pitch_deg, g_yaw_deg, data_roll_pitch_yaw);// 构建 MPU 角度数据

    ss_send(Serial, FUNC_ODOM_COUNTS, data_counts, false);// 发送轮速编码器数据
    ss_send(Serial, FUNC_ODOM_SPEED, data_speed, false);// 发送轮速编码器速度数据
    ss_send(Serial, FUNC_ODOM_DISTANCE, data_distance, false);// 发送超声波传感器距离数据
    ss_send(Serial, FUNC_IMU_ROLL_PITCH_YAW, data_roll_pitch_yaw, false);// 发送 MPU 角度数据
  }
}
