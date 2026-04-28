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
    delay(100);

    // 唤醒
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(50);

    // 读取设备ID 0x70
    Wire.beginTransmission(_mpu_addr);
    Wire.write(0x75);
    Wire.endTransmission();
    Wire.requestFrom(_mpu_addr, 1);
    uint8_t id = Wire.read();
    if(id != 0x70)
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
    Wire.requestFrom(_mpu_addr, 14);

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

// 更新：读原始数据 + 卡尔曼解算角度
  void mpu6050_update(void)
{
    float dt = (millis() - _lasttime) / 1000.0f;
    _lasttime = millis();
    if (dt <= 0.0f || dt > 0.2f) dt = 0.0f;

      mpu6050_read_raw();

    // 单位换算 + 偏移补偿
    float axf = (ax - ax_offset) / 16384.0f;
    float ayf = (ay - ay_offset) / 16384.0f;
    float azf = az / 16384.0f;

    float gxf = (gx - gx_offset) / 131.0f;
    float gyf = (gy - gy_offset) / 131.0f;
    float gzf = (gz - gz_offset) / 131.0f;

    if (dt > 0.0f) {
        const bool is_still = (fabsf(axf) < 0.05f) && (fabsf(ayf) < 0.05f) && (fabsf(azf - 1.0f) < 0.05f) &&
                              (fabsf(gxf) < 0.6f) && (fabsf(gyf) < 0.6f) && (fabsf(gzf) < 0.6f);
        if (is_still) {
            constexpr float alpha = 0.01f;
            gz_bias_dps = (1.0f - alpha) * gz_bias_dps + alpha * gzf;
        }
        gzf -= gz_bias_dps;
    }

    // 陀螺仪姿态积分
    float roll_v  = gxf + (sin(pitch)*sin(roll)/cos(pitch))*gyf + (sin(pitch)*cos(roll)/cos(pitch))*gzf;
    float pitch_v = cos(roll)*gyf - sin(roll)*gzf;

    gyro_roll  = roll + dt * roll_v;
    gyro_pitch = pitch + dt * pitch_v;

    // 协方差更新
    e_P[0][0] += 0.0025f;
    e_P[1][1] += 0.0025f;

    float k0 = e_P[0][0] / (e_P[0][0] + 0.3f);
    float k1 = e_P[1][1] / (e_P[1][1] + 0.3f);

    // 加速度计观测角度
    acc_roll  = atan(ayf / azf) * rad2deg;
    acc_pitch = -atan(axf / sqrt(ayf*ayf + azf*azf)) * rad2deg;

    // 卡尔曼融合
    roll  = gyro_roll  + k0 * (acc_roll - gyro_roll);
    pitch = gyro_pitch + k1 * (acc_pitch - gyro_pitch);

    // 协方差修正
    e_P[0][0] = (1.0f - k0) * e_P[0][0];
    e_P[1][1] = (1.0f - k1) * e_P[1][1];

    // Yaw 纯陀螺积分（无磁传感器必然漂移）
    yaw += gzf * dt;
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

void mpu6050_pack_accel_g_x1000(float accel_x_g, float accel_y_g, float accel_z_g, uint8_t out6[6])
{
    const int16_t ax_out = clamp_i16(static_cast<int32_t>(accel_x_g * 1000.0f));
    const int16_t ay_out = clamp_i16(static_cast<int32_t>(accel_y_g * 1000.0f));
    const int16_t az_out = clamp_i16(static_cast<int32_t>(accel_z_g * 1000.0f));
    write_i16_le(&out6[0], ax_out);
    write_i16_le(&out6[2], ay_out);
    write_i16_le(&out6[4], az_out);
}

void mpu6050_pack_gyro_dps_x10(float gyro_x_dps, float gyro_y_dps, float gyro_z_dps, uint8_t out6[6])
{
    const int16_t gx_out = clamp_i16(static_cast<int32_t>(gyro_x_dps * 10.0f));
    const int16_t gy_out = clamp_i16(static_cast<int32_t>(gyro_y_dps * 10.0f));
    const int16_t gz_out = clamp_i16(static_cast<int32_t>(gyro_z_dps * 10.0f));
    write_i16_le(&out6[0], gx_out);
    write_i16_le(&out6[2], gy_out);
    write_i16_le(&out6[4], gz_out);
}
