#pragma once
#include <Arduino.h>

bool mpu6050_init();
void mpu6050_read(double *ax, double *ay, double *az, double *gx, double *gy, double *gz);

bool mpu6050_pack_accel_g_x1000(double accel_x, double accel_y, double accel_z, uint8_t out6[6]);
bool mpu6050_pack_gyro_dps_x10(double gyro_x, double gyro_y, double gyro_z, uint8_t out6[6]);
