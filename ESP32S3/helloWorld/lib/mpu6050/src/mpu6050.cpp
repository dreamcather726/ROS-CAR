#include "mpu6050.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

bool mpu6050_init() {
  if (!mpu.begin()) {
    return false;
  }

  // 设置量程，和你原来的逻辑一致
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  return true;
}

void mpu6050_read(double *ax, double *ay, double *az, double *gx, double *gy, double *gz) {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // 转换为 g（重力加速度）
  *ax = a.acceleration.x / 9.81;
  *ay = a.acceleration.y / 9.81;
  *az = a.acceleration.z / 9.81;

  // 转换为 °/s
  *gx = g.gyro.x * 57.2958;
  *gy = g.gyro.y * 57.2958;
  *gz = g.gyro.z * 57.2958;
}

static inline void write_i16_le(uint8_t *out, int16_t v) {
  out[0] = static_cast<uint8_t>(v & 0xFF);
  out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

bool mpu6050_pack_accel_g_x1000(double accel_x, double accel_y, double accel_z, uint8_t out6[6]) {
  const int16_t ax = clamp_i16(static_cast<int32_t>(accel_x * 1000.0));
  const int16_t ay = clamp_i16(static_cast<int32_t>(accel_y * 1000.0));
  const int16_t az = clamp_i16(static_cast<int32_t>(accel_z * 1000.0));
  write_i16_le(&out6[0], ax);
  write_i16_le(&out6[2], ay);
  write_i16_le(&out6[4], az);
  return true;
}

bool mpu6050_pack_gyro_dps_x10(double gyro_x, double gyro_y, double gyro_z, uint8_t out6[6]) {
  const int16_t gx = clamp_i16(static_cast<int32_t>(gyro_x * 10.0));
  const int16_t gy = clamp_i16(static_cast<int32_t>(gyro_y * 10.0));
  const int16_t gz = clamp_i16(static_cast<int32_t>(gyro_z * 10.0));
  write_i16_le(&out6[0], gx);
  write_i16_le(&out6[2], gy);
  write_i16_le(&out6[4], gz);
  return true;
}