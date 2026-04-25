
#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <serial_sender.h>
#include <odom_timer.h>
#include <ultrasonic.h>
#include <odom_frame.h>
#include <mpu6050.h>
#include "speed_pid.h"
#include <button.h>

Button btn_add;
Button btn_sub;

#define SERIAL_BAUD 115200

// PID 周期 100ms（必须固定！）
static constexpr uint32_t ODOM_PERIOD_US = 100000U;

// 只使用左轮 PID
static PID pidL;
float target_cm_s = 30.0f;
void setup() {
  Serial.begin(SERIAL_BAUD);
  button_init(&btn_add, 9, true, true, 30);
  button_init(&btn_sub, 10, true, true, 30);
  ultrasonic_init();
  wheel_encoder_init();
  mpu6050_init();
  wheel_encoder_speed_init();
  motor_init();
  odom_timer_init(ODOM_PERIOD_US);

  // ====================== 关键：必须加 Ki ======================
  // PID_Init(&pidL, 4.0f, 0.5f, 0.0f, 255.0f, -255.0f);
  PID_Init(&pidL, pidL.P+0.5f, pidL.I+1.4f, pidL.D+0.2f, 255.0f, -255.0f);
}
//PID循环
void loop() {

  
  // 1. 每 100ms 执行一次 PID（固定周期！）
  if (!odom_timer_due()) {
    return;
  }

  // 2. 获取速度
  float left_cm_s = 0.0f;
  float right_cm_s = 0.0f;
  int32_t left_count = 0;
  int32_t right_count = 0;
  if (!wheel_encoder_get_odom(&left_count, &right_count, &left_cm_s, &right_cm_s, ODOM_PERIOD_US)) {
    return;
  }

  // 3. 目标速度
 

  // 4. PID 计算
  float pwm_float = PID_IncPIDCal(&pidL, left_cm_s, target_cm_s);
  
  // 5. 安全限幅
  int pwmL = constrain((int)pwm_float, -255, 255);

  // 6. 输出到电机
  motorA_set(pwmL);

  // 调试输出

  Serial.print(target_cm_s);
  Serial.print(",");
  Serial.print(left_cm_s);
  Serial.print(",");
  Serial.println(pwmL);
}


// void loop() {
//   motorA_set(128);
//   motorB_set(128);

//   if (!odom_timer_due()) return;

//   int32_t left_count = 0;
//   int32_t right_count = 0;
//   float distance = ultrasonic_get_distance();
//   float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
//   mpu6050_read(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
  
//   float left_cm_s = 0.0f;
//   float right_cm_s = 0.0f;
//   if (!wheel_encoder_get_odom(&left_count, &right_count, &left_cm_s, &right_cm_s, ODOM_PERIOD_US)) return;

//   if (Serial.availableForWrite() < static_cast<int>(SERIAL_SENDER_FRAME_SIZE * 5)) return;

//   uint8_t data_counts[SERIAL_SENDER_DATA_SIZE];// 里程计计数
//   uint8_t data_speed[SERIAL_SENDER_DATA_SIZE];// 里程计速度
//   uint8_t data_distance[SERIAL_SENDER_DATA_SIZE];// 里程计距离
//   uint8_t data_accel[SERIAL_SENDER_DATA_SIZE];// IMU加速度
//   uint8_t data_gyro[SERIAL_SENDER_DATA_SIZE];// IMU陀螺仪

//   odom_frame_build_counts_i24_le(left_count, right_count, data_counts);// 里程计计数
//   odom_frame_build_speed_i16_le_x100(left_cm_s, right_cm_s, data_speed);// 里程计速度
//   ultrasonic_pack_distance_cm_x10(distance, data_distance);// 里程计距离
//   mpu6050_pack_accel_g_x1000(accel_x, accel_y, accel_z, data_accel);// IMU加速度
//   mpu6050_pack_gyro_dps_x10(gyro_x, gyro_y, gyro_z, data_gyro);// IMU陀螺仪

//   ss_send(Serial, FUNC_ODOM_COUNTS, data_counts, false);// 里程计计数
//   ss_send(Serial, FUNC_ODOM_SPEED, data_speed, false);// 里程计速度
//   ss_send(Serial, FUNC_ODOM_DISTANCE, data_distance, false);// 里程计距离
//   ss_send(Serial, FUNC_IMU_ACCEL, data_accel, false);// IMU加速度
//   ss_send(Serial, FUNC_IMU_GYRO, data_gyro, false);// IMU陀螺仪
//   //原始数据
//   // Serial.println("原始数据：");
  
//   // Serial.println(left_count);

//   // Serial.println(right_count);
//   // Serial.println(distance);
//   // Serial.println(accel_x);
//   // Serial.println(accel_y);
//   // Serial.println(accel_z);
//   // Serial.println(gyro_x);
//   // Serial.println(gyro_y);
//   // Serial.println(gyro_z);
//   // delay(1000);
// }
