#include "mpu6050.h"

// 全局变量定义
int16_t ax, ay, az;
int16_t gx, gy, gz;

float roll  = 0.0f;
float pitch = 0.0f;
float yaw   = 0.0f;

float ax_offset = 0.0f;
float ay_offset = 0.0f;
float gx_offset = 0.0f;
float gy_offset = 0.0f;
float gz_offset = 0.0f;

static uint8_t  _mpu_addr;
static unsigned long _lasttime = 0;
static float gyro_roll = 0.0f, gyro_pitch = 0.0f;
static float acc_roll  = 0.0f, acc_pitch  = 0.0f;
static float e_P[2][2] = {{1,0},{0,1}};
static float gz_bias_dps = 0.0f;

// 初始化+校验ID
bool mpu6050_init(uint8_t addr)
{
    _mpu_addr = addr;
    Wire.begin(I2C_SDA_PIN,I2C_SCL_PIN);
    Wire.setClock(100000);

    // 唤醒
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

   
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom(static_cast<uint8_t>(_mpu_addr), static_cast<uint8_t>(1));
    uint8_t id = Wire.read();
    if(id != 0x68)
    {
        return false;
    }

    // 配置量程、滤波
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x1B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x1C);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x1A);
    Wire.write(0x04);
    Wire.endTransmission();

    _lasttime = millis();
    return true;
}

// 静态读取原始寄存器数据
static void mpu6050_read_raw(void)
{
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<uint8_t>(_mpu_addr), static_cast<uint8_t>(14));

    ax = Wire.read() << 8 | Wire.read();
    ay = Wire.read() << 8 | Wire.read();
    az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read();
    gx = Wire.read() << 8 | Wire.read();
    gy = Wire.read() << 8 | Wire.read();
    gz = Wire.read() << 8 | Wire.read();
}

// 零点校准
void mpu6050_calibrate(void)
{
    Serial.println("校准中，请保持传感器静止...");
    long sum_ax = 0, sum_ay = 0;
    long sum_gx = 0, sum_gy = 0, sum_gz = 0;

    for(uint16_t i = 0; i < 2000; i++)
    {
        mpu6050_read_raw();
        sum_ax += ax;
        sum_ay += ay;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;
        delay(1);
    }

    ax_offset = sum_ax / 2000.0f;
    ay_offset = sum_ay / 2000.0f;
    gx_offset = sum_gx / 2000.0f;
    gy_offset = sum_gy / 2000.0f;
    gz_offset = sum_gz / 2000.0f;

    Serial.println("校准完成！");
}

void mpu6050_update(void)
{
    const uint32_t now_ms = millis();
    float dt = static_cast<float>(now_ms - _lasttime) / 1000.0f;
    _lasttime = now_ms;
    if (dt <= 0.0f) dt = 0.0f;
    if (dt > 0.2f) dt = 0.02f;          // 防止异常间隔过大

    // 读取原始数据
    mpu6050_read_raw();

    // 单位换算及偏移补偿
    float axf = (ax - ax_offset) / 16384.0f;
    float ayf = (ay - ay_offset) / 16384.0f;
    float azf = az / 16384.0f;           // 若需要 az_offset 可自行添加

    float gxf = (gx - gx_offset) / 131.0f;
    float gyf = (gy - gy_offset) / 131.0f;
    float gzf = (gz - gz_offset) / 131.0f;

    // 静止时动态更新 yaw 轴陀螺仪零偏（可选）
    if (dt > 0.0f) {
        const float amag = sqrtf(axf * axf + ayf * ayf + azf * azf);
        const bool accel_still = fabsf(amag - 1.0f) < 0.12f;
        const bool gyro_still = (fabsf(gxf) < 1.2f) && (fabsf(gyf) < 1.2f) && (fabsf(gzf) < 1.2f);
        const bool is_still = accel_still && gyro_still;
        if (is_still) {
            const float alpha = 0.02f;
            gz_bias_dps = (1.0f - alpha) * gz_bias_dps + alpha * gzf;
        }
        gzf -= gz_bias_dps;   // 补偿后的偏航角速度
    }

    // --- 陀螺仪积分（欧拉角微分方程）---
    const float roll_rad  = roll * deg2rad;
    const float pitch_rad = pitch * deg2rad;
    float cos_pitch = cosf(pitch_rad);
    if (fabsf(cos_pitch) < 1e-4f) cos_pitch = (cos_pitch >= 0.0f) ? 1e-4f : -1e-4f;

    // 将机体角速度转换为欧拉角速度
    float roll_v  = gxf + (sinf(pitch_rad) * sinf(roll_rad) / cos_pitch) * gyf +
                    (sinf(pitch_rad) * cosf(roll_rad) / cos_pitch) * gzf;
    float pitch_v = cosf(roll_rad) * gyf - sinf(roll_rad) * gzf;

    // 先验估计（仅陀螺仪积分）
    float gyro_roll_new  = roll  + dt * roll_v;
    float gyro_pitch_new = pitch + dt * pitch_v;

    // --- 加速度计计算角度（绝对参考）---
    float acc_roll_new  = atan2f(ayf, azf) * rad2deg;
    float acc_pitch_new = -atan2f(axf, sqrtf(ayf * ayf + azf * azf)) * rad2deg;

    // --- 卡尔曼融合（roll 和 pitch 独立）---
    // 协方差预测（增加过程噪声）
    e_P[0][0] += 0.0025f;
    e_P[1][1] += 0.0025f;

    // 卡尔曼增益
    float k0 = e_P[0][0] / (e_P[0][0] + 0.3f);
    float k1 = e_P[1][1] / (e_P[1][1] + 0.3f);

    // 融合
    roll  = gyro_roll_new  + k0 * (acc_roll_new  - gyro_roll_new);
    pitch = gyro_pitch_new + k1 * (acc_pitch_new - gyro_pitch_new);

    // 协方差更新
    e_P[0][0] = (1.0f - k0) * e_P[0][0];
    e_P[1][1] = (1.0f - k1) * e_P[1][1];

    // --- 偏航角：纯陀螺积分（无磁力计修正，必然漂移）---
    yaw += gzf * dt;

    // 可选：限制 yaw 范围到 [0, 360) 或 [-180, 180)
    if (yaw > 360.0f) yaw -= 360.0f;
    if (yaw < 0.0f)   yaw += 360.0f;
}
