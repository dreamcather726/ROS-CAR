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

static inline void write_i16_le(uint8_t *out, int16_t v)
{
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline int16_t clamp_i16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

bool mpu6050_pack_accel_g_x1000(float accel_x, float accel_y, float accel_z, uint8_t out6[6])
{
  const int16_t ax = clamp_i16(static_cast<int32_t>(accel_x * 1000.0f));
  const int16_t ay = clamp_i16(static_cast<int32_t>(accel_y * 1000.0f));
  const int16_t az = clamp_i16(static_cast<int32_t>(accel_z * 1000.0f));
  write_i16_le(&out6[0], ax);
  write_i16_le(&out6[2], ay);
  write_i16_le(&out6[4], az);
  return true;
}

bool mpu6050_pack_gyro_dps_x10(float gyro_x, float gyro_y, float gyro_z, uint8_t out6[6])
{
  const int16_t gx = clamp_i16(static_cast<int32_t>(gyro_x * 10.0f));
  const int16_t gy = clamp_i16(static_cast<int32_t>(gyro_y * 10.0f));
  const int16_t gz = clamp_i16(static_cast<int32_t>(gyro_z * 10.0f));
  write_i16_le(&out6[0], gx);
  write_i16_le(&out6[2], gy);
  write_i16_le(&out6[4], gz);
  return true;
}
