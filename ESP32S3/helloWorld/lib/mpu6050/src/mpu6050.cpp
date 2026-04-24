#include "mpu6050.h"
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

bool mpu6050_init() {
  Wire.begin();
  
  if (!mpu.initialize()) {
    return false;
  }
  if (!mpu.testConnection()) {
    return false;
  }

  // 设置量程
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
  return true;
}

void mpu6050_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx_raw, &gy_raw, &gz_raw);

  // 转换单位
  *ax = ax_raw / 16384.0f;
  *ay = ay_raw / 16384.0f;
  *az = az_raw / 16384.0f;

  *gx = gx_raw / 16.4f;
  *gy = gy_raw / 16.4f;
  *gz = gz_raw / 16.4f;
}