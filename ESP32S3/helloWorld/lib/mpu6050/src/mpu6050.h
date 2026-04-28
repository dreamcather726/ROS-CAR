
#include <Arduino.h>
#include <Wire.h>

// 可自行修改I2C地址
#define MPU6050_ADDR    0x68
#define rad2deg         57.29578f
#define I2C_SDA_PIN       8
#define I2C_SCL_PIN       9
// 原始数据 全局变量
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

// 解算姿态角 全局变量
extern float roll, pitch, yaw;

// 偏移校准
extern float ax_offset, ay_offset;
extern float gx_offset, gy_offset, gz_offset;

// 函数声明
bool mpu6050_init(uint8_t addr = MPU6050_ADDR);
void mpu6050_calibrate(void);
void mpu6050_update(void);

void mpu6050_pack_accel_g_x1000(float accel_x_g, float accel_y_g, float accel_z_g, uint8_t out6[6]);
void mpu6050_pack_gyro_dps_x10(float gyro_x_dps, float gyro_y_dps, float gyro_z_dps, uint8_t out6[6]);
