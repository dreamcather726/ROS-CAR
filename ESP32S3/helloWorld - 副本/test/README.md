| 模块 | 信号 | ESP32-S3 GPIO | 备注 |
|---|---|---:|---|
| 串口(外设) | TX | 43 | 原理图 TX |
| 串口(外设) | RX | 44 | 原理图 RX |
| 右电机驱动 | PWMA | 4 | RIGHT_MOTOR_PWM_PIN，set_right_motor_pwm |
| 右电机驱动 | AN1 | 5 | RIGHT_MOTOR_DIR_PIN，set_right_motor_pwm |
| 左电机驱动 | PWMB | 1 | LEFT_MOTOR_PWM_PIN，set_left_motor_pwm |
| 左电机驱动 | BN1 | 2 | LEFT_MOTOR_DIR_PIN，set_left_motor_pwm |
| 编码器-左 | A+ | 6 | WHEEL_ENC_L_A_PIN |
| 编码器-左 | A- | 7 | WHEEL_ENC_L_B_PIN |
| 编码器-右 | B+ | 42 | WHEEL_ENC_R_A_PIN |
| 编码器-右 | B- | 41 | WHEEL_ENC_R_B_PIN |
| MPU6050(I2C) | SDA | 8 | I2C_SDA_PIN |
| MPU6050(I2C) | SCL | 9 | I2C_SCL_PIN |
