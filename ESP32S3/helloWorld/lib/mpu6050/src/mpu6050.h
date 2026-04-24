#pragma once
#include <Arduino.h>

bool mpu6050_init();
void mpu6050_read(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);

void mpu6050_pack_accel_g_x1000(float accel_x, float accel_y, float accel_z, uint8_t out6[6]);
void mpu6050_pack_gyro_dps_x10(float gyro_x, float gyro_y, float gyro_z, uint8_t out6[6]);
