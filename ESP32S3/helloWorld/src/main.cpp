#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <serial_sender.h>
#include <odom_timer.h>
#include <ultrasonic.h>
#include <odom_frame.h>
#include <mpu6050.h>

#define SERIAL_BAUD 115200

static constexpr uint8_t FUNC_ODOM_COUNTS = 0x10;
static constexpr uint8_t FUNC_ODOM_SPEED = 0x11;
static constexpr uint8_t FUNC_ODOM_DISTANCE = 0x12;
static constexpr uint8_t FUNC_IMU_ACCEL = 0x20;
static constexpr uint8_t FUNC_IMU_GYRO = 0x21;

static constexpr uint32_t ODOM_PERIOD_US = 100000U;// 100ms

void setup() {
  Serial.begin(SERIAL_BAUD);
  ultrasonic_init();
  wheel_encoder_init();
  mpu6050_init(); // 初始化MPU6050
  wheel_encoder_speed_init();
  motor_init();
  odom_timer_init(ODOM_PERIOD_US);
}

void loop() {
  motorA_set(128);
  motorB_set(128);

  if (!odom_timer_due()) return;

  int32_t left_count = 0;
  int32_t right_count = 0;
  float distance = ultrasonic_get_distance();
  float accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z;
  mpu6050_read(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
  
  float left_cm_s = 0.0f;
  float right_cm_s = 0.0f;
  if (!wheel_encoder_get_odom(&left_count, &right_count, &left_cm_s, &right_cm_s, ODOM_PERIOD_US)) return;

  if (Serial.availableForWrite() < static_cast<int>(SERIAL_SENDER_FRAME_SIZE * 5)) return;

  uint8_t data_counts[SERIAL_SENDER_DATA_SIZE];
  uint8_t data_speed[SERIAL_SENDER_DATA_SIZE];
  uint8_t data_distance[SERIAL_SENDER_DATA_SIZE];
  uint8_t data_accel[SERIAL_SENDER_DATA_SIZE];
  uint8_t data_gyro[SERIAL_SENDER_DATA_SIZE];

  if (!odom_frame_build_counts_i24_le(left_count, right_count, data_counts)) return;
  if (!odom_frame_build_speed_i16_le_x100(left_cm_s, right_cm_s, data_speed)) return;
  if (!ultrasonic_pack_distance_cm_x10(distance, data_distance)) return;
  if (!mpu6050_pack_accel_g_x1000(accel_x, accel_y, accel_z, data_accel)) return;
  if (!mpu6050_pack_gyro_dps_x10(gyro_x, gyro_y, gyro_z, data_gyro)) return;

  ss_send(Serial, FUNC_ODOM_COUNTS, data_counts, false);
  ss_send(Serial, FUNC_ODOM_SPEED, data_speed, false);
  ss_send(Serial, FUNC_ODOM_DISTANCE, data_distance, false);
  ss_send(Serial, FUNC_IMU_ACCEL, data_accel, false);
  ss_send(Serial, FUNC_IMU_GYRO, data_gyro, false);
}
