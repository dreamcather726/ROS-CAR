#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"  // 官方库 DMP 支持

// 默认 I2C 引脚（ESP32 常用），可在调用 init 前修改
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 19
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 20
#endif

// 全局变量声明（供外部使用）
extern float q_w, q_x, q_y, q_z;  // 四元数
extern float yaw, pitch, roll;    // 欧拉角（度）
extern int16_t ax, ay, az;        // 原始加速度计数据
extern int16_t gx, gy, gz;        // 原始陀螺仪数据

// 函数声明
bool mpu6050_init(uint8_t addr = 0x68);
void mpu6050_calibrate(void);
void mpu6050_update(void);
