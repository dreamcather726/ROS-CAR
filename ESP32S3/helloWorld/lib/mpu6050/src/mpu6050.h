
#pragma once

#include <Arduino.h>

// 初始化MPU6050
// 返回：true=成功，false=失败
bool mpu6050_init();

// 读取六轴数据
void mpu6050_read(
  float *accel_x,  // g
  float *accel_y,
  float *accel_z,
  float *gyro_x,   // °/s
  float *gyro_y,
  float *gyro_z
);

bool mpu6050_pack_accel_g_x1000(float accel_x, float accel_y, float accel_z, uint8_t out6[6]);
bool mpu6050_pack_gyro_dps_x10(float gyro_x, float gyro_y, float gyro_z, uint8_t out6[6]);
