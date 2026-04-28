
#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <serial_sender.h>
#include <ultrasonic.h>
#include <odom_frame.h>
#include <mpu6050.h>
#include "speed_pid.h"
#include <button.h>
#include <odom_timer.h>

Button btn_add;
Button btn_sub;

#define SERIAL_BAUD 115200

static constexpr uint8_t FUNC_ODOM_COUNTS = 0x01;// 0x01: 速度计数
static constexpr uint8_t FUNC_ODOM_SPEED = 0x02;// 0x02: 速度
static constexpr uint8_t FUNC_ODOM_DISTANCE = 0x03;// 0x03: 距离
static constexpr uint8_t FUNC_IMU_ACCEL = 0x04;// 0x04: 加速度
static constexpr uint8_t FUNC_IMU_GYRO = 0x05;// 0x05: 陀螺仪

static constexpr uint32_t BUTTON_DEBOUNCE_MS = 10U;// 10ms 软抖动
static constexpr bool SEND_TO_MUSEPI = false;// 发送到 MUSEPI
static constexpr bool DEBUG_PID_PRINT = false;// 打印 PID 值

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
float left_cm_s = 0.0f;
float right_cm_s = 0.0f;
float pwmL_float = 0.0f;
float pwmR_float = 0.0f;
int pwmL = 0;
int pwmR = 0;
static int32_t g_left_count = 0;// 左轮速度计数器
static int32_t g_right_count = 0;// 右轮速度计数器
static float g_left_cm_s = 0.0f;// 左轮速度
static float g_right_cm_s = 0.0f;// 右轮速度
static float g_ultrasonic_cm = -1.0f;// 超声波距离
static float g_accel_x = 0.0f;// 加速度 X 轴  
static float g_accel_y = 0.0f;// 加速度 Y 轴
static float g_accel_z = 0.0f;// 加速度 Z 轴
static float g_gyro_x = 0.0f;// 陀螺仪 X 轴
static float g_gyro_y = 0.0f;// 陀螺仪 Y 轴
static float g_gyro_z = 0.0f;// 陀螺仪 Z 轴
static float g_roll_deg = 0.0f;
static float g_pitch_deg = 0.0f;
static float g_yaw_deg = 0.0f;
void setup() {
  Serial.begin(SERIAL_BAUD);
  button_init(&btn_add, 10, true, true, BUTTON_DEBOUNCE_MS);// 加速按钮
  button_init(&btn_sub, 11, true, true, BUTTON_DEBOUNCE_MS);// 减速按钮
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
  if (button_update_pressed(&btn_add)) {
    target_cm_s += 10.0f;
  }
  if (button_update_pressed(&btn_sub)) {
    target_cm_s -= 10.0f;
  }

  if (timer_due(&pid_timer)) {
    wheel_encoder_get_odom(&g_left_count, &g_right_count, &g_left_cm_s, &g_right_cm_s, 0U);
    left_cm_s = g_left_cm_s;
    right_cm_s = g_right_cm_s;
    pwmL_float = PID_IncPIDCal(&pidL, left_cm_s, target_cm_s);
    pwmR_float = PID_IncPIDCal(&pidR, right_cm_s, target_cm_s);
    pwmL = constrain((int)pwmL_float, -255, 255); 
    motorA_set(pwmL);
    motorB_set(pwmR);
    
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

  if (timer_due(&sensor_read_timer)) {
    g_ultrasonic_cm = ultrasonic_get_distance();
    mpu6050_update();
    g_accel_x = (static_cast<float>(ax) - ax_offset) / 16384.0f;
    g_accel_y = (static_cast<float>(ay) - ay_offset) / 16384.0f;
    g_accel_z = static_cast<float>(az) / 16384.0f;
    g_gyro_x = (static_cast<float>(gx) - gx_offset) / 131.0f;
    g_gyro_y = (static_cast<float>(gy) - gy_offset) / 131.0f;
    g_gyro_z = (static_cast<float>(gz) - gz_offset) / 131.0f;
    g_roll_deg = roll;
    g_pitch_deg = pitch;
    g_yaw_deg = yaw;
    }
      Serial.print(g_accel_x);
      Serial.print(",");
      Serial.print(g_accel_y);
      Serial.print(",");
      Serial.print(g_accel_z);
      Serial.print(",");
      Serial.print(g_gyro_x);
      Serial.print(",");
      Serial.print(g_gyro_y);
      Serial.print(",");
      Serial.print(g_gyro_z);
      Serial.print(",");
      Serial.print(g_roll_deg);
      Serial.print(",");
      Serial.print(g_pitch_deg);
      Serial.print(",");
      Serial.println(g_yaw_deg);

    

  if (SEND_TO_MUSEPI && timer_due(&tx_timer)) {
    if (Serial.availableForWrite() < static_cast<int>(SERIAL_SENDER_FRAME_SIZE * 5)) {
      return;
    }

    uint8_t data_counts[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_speed[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_distance[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_accel[SERIAL_SENDER_DATA_SIZE];
    uint8_t data_gyro[SERIAL_SENDER_DATA_SIZE];

    odom_frame_build_counts_i24_le(g_left_count, g_right_count, data_counts);
    odom_frame_build_speed_i16_le_x100(g_left_cm_s, g_right_cm_s, data_speed);
    ultrasonic_pack_distance_cm_x10(g_ultrasonic_cm, data_distance);
    mpu6050_pack_accel_g_x1000(g_accel_x, g_accel_y, g_accel_z, data_accel);
    mpu6050_pack_gyro_dps_x10(g_gyro_x, g_gyro_y, g_gyro_z, data_gyro);

    ss_send(Serial, FUNC_ODOM_COUNTS, data_counts, false);
    ss_send(Serial, FUNC_ODOM_SPEED, data_speed, false);
    ss_send(Serial, FUNC_ODOM_DISTANCE, data_distance, false);
    ss_send(Serial, FUNC_IMU_ACCEL, data_accel, false);
    ss_send(Serial, FUNC_IMU_GYRO, data_gyro, false);
  }
}
