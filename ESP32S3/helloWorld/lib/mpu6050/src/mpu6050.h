
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