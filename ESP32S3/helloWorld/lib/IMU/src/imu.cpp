#include "imu.h"

// 全局变量定义
float q_w = 1.0f, q_x = 0.0f, q_y = 0.0f, q_z = 0.0f;
float yaw = 0.0f, pitch = 0.0f, roll = 0.0f;
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

static MPU6050 mpu;
static bool dmp_ready = false;
static uint8_t fifo_buffer[64];
static uint16_t packet_size;
static uint8_t mpu_addr = 0x68;
static bool ypr_zero_inited = false;
static float yaw_zero = 0.0f;
static float pitch_zero = 0.0f;
static float roll_zero = 0.0f;

// 读取原始传感器数据（加速度 + 陀螺仪）
static void read_raw() {
    Wire.beginTransmission(mpu_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(static_cast<uint8_t>(mpu_addr), static_cast<uint8_t>(14));
    ax = (int16_t)((Wire.read() << 8) | Wire.read());
    ay = (int16_t)((Wire.read() << 8) | Wire.read());
    az = (int16_t)((Wire.read() << 8) | Wire.read());
    Wire.read(); Wire.read(); // 跳过温度数据
    gx = (int16_t)((Wire.read() << 8) | Wire.read());
    gy = (int16_t)((Wire.read() << 8) | Wire.read());
    gz = (int16_t)((Wire.read() << 8) | Wire.read());
}

// 初始化传感器并启动 DMP
bool mpu6050_init(uint8_t addr) {
    mpu_addr = addr;
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(400000);  // 400kHz I2C 时钟

    mpu.initialize();
    if (!mpu.testConnection()) {
        return false;
    }

    // 加载并配置 DMP
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus != 0) {
        return false;
    }

    // 可选：设置数字低通滤波器 (DLPF) 等，此处使用默认

    packet_size = mpu.dmpGetFIFOPacketSize();
    mpu.setDMPEnabled(true);
    dmp_ready = true;
    return true;
}

// 校准加速度计和陀螺仪（使用库自带校准函数）
void mpu6050_calibrate(void) {
    if (!dmp_ready) return;
    mpu.CalibrateAccel(10);//校准加速度计偏移量
    mpu.CalibrateGyro(10);//校准陀螺仪偏移量
    mpu.setDMPEnabled(true);
    mpu.resetFIFO();
    ypr_zero_inited = false;

    float yaw_unwrapped_deg = 0.0f;
    float last_yaw_deg = 0.0f;
    float last_pitch_deg = 0.0f;
    float last_roll_deg = 0.0f;
    bool has_first = false;

    const uint32_t start_ms = millis();
    while (millis() - start_ms < 20000U) {
        if (!mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
            delay(5);
            continue;
        }

        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        const float yaw_deg = ypr[0] * 180.0f / M_PI;
        const float pitch_deg = ypr[1] * 180.0f / M_PI;
        const float roll_deg = ypr[2] * 180.0f / M_PI;

        if (!has_first) {
            has_first = true;
            yaw_unwrapped_deg = yaw_deg;
            last_yaw_deg = yaw_deg;
        } else {
            float d = yaw_deg - last_yaw_deg;
            while (d > 180.0f) d -= 360.0f;
            while (d < -180.0f) d += 360.0f;
            yaw_unwrapped_deg += d;
            last_yaw_deg = yaw_deg;
        }

        last_pitch_deg = pitch_deg;
        last_roll_deg = roll_deg;
        delay(5);
    }

    if (!has_first) {
        yaw_zero = 0.0f;
        pitch_zero = 0.0f;
        roll_zero = 0.0f;
        ypr_zero_inited = true;
        yaw = 0.0f;
        pitch = 0.0f;
        roll = 0.0f;
        return;
    }

    yaw_zero = yaw_unwrapped_deg;
    pitch_zero = last_pitch_deg;
    roll_zero = last_roll_deg;
    ypr_zero_inited = true;

    yaw = 0.0f;
    pitch = 0.0f;
    roll = 0.0f;
}

// 更新姿态数据（必须在 loop 中高频调用）
void mpu6050_update(void) {
    if (!dmp_ready) return;

    // 尝试从 FIFO 读取一个完整的数据包
    if (mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        Quaternion q;         // 库定义的四元数结构体 {w,x,y,z}
        VectorFloat gravity;  // 重力向量
        float ypr[3];         // yaw/pitch/roll（弧度）

        mpu.dmpGetQuaternion(&q, fifo_buffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        q_w = q.w;
        q_x = q.x;
        q_y = q.y;
        q_z = q.z;

        yaw   = ypr[0] * 180.0f / M_PI;
        pitch = ypr[1] * 180.0f / M_PI;
        roll  = ypr[2] * 180.0f / M_PI;

        if (ypr_zero_inited) {
            yaw -= yaw_zero;
            pitch -= pitch_zero;
            roll -= roll_zero;

            yaw = fmodf(yaw, 360.0f);
            if (yaw < 0.0f) yaw += 360.0f;
        }
    }

    // 读取原始数据（用于需要原始值的场景）
    read_raw();
}
