#include "mpu6050.h"
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

static Adafruit_MPU6050 g_mpu;

bool mpu6050_init() {
  Wire.begin();
  
  if (!g_mpu.begin()) {
    return false;
  }

  g_mpu.setGyroRange(MPU6050_RANGE_2000_DEG);
  g_mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  g_mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  return true;
}

void mpu6050_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
  sensors_event_t a, g, t;
  g_mpu.getEvent(&a, &g, &t);

  const float inv_g = 1.0f / 9.80665f;
  if (ax) *ax = a.acceleration.x * inv_g;
  if (ay) *ay = a.acceleration.y * inv_g;
  if (az) *az = a.acceleration.z * inv_g;

  const float rad_to_deg = 180.0f / PI;
  if (gx) *gx = g.gyro.x * rad_to_deg;
  if (gy) *gy = g.gyro.y * rad_to_deg;
  if (gz) *gz = g.gyro.z * rad_to_deg;
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

void mpu6050_pack_accel_g_x1000(float accel_x, float accel_y, float accel_z, uint8_t out6[6])
{
  const int16_t ax = clamp_i16(static_cast<int32_t>(accel_x * 1000.0f));
  const int16_t ay = clamp_i16(static_cast<int32_t>(accel_y * 1000.0f));
  const int16_t az = clamp_i16(static_cast<int32_t>(accel_z * 1000.0f));
  write_i16_le(&out6[0], ax);
  write_i16_le(&out6[2], ay);
  write_i16_le(&out6[4], az);
}

void mpu6050_pack_gyro_dps_x10(float gyro_x, float gyro_y, float gyro_z, uint8_t out6[6])
{
  const int16_t gx = clamp_i16(static_cast<int32_t>(gyro_x * 10.0f));
  const int16_t gy = clamp_i16(static_cast<int32_t>(gyro_y * 10.0f));
  const int16_t gz = clamp_i16(static_cast<int32_t>(gyro_z * 10.0f));
  write_i16_le(&out6[0], gx);
  write_i16_le(&out6[2], gy);
  write_i16_le(&out6[4], gz);
}
