#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include "speed_pid.h"
#include "serial_sender.h"
#include <Ticker.h>
#include "imu.h"

// 串口发送功能码：编码器总计数
static constexpr uint8_t FRAME_FUNC_ENCODER = 0x01;
static constexpr uint8_t FRAME_FUNC_IMU = 0x02;

// 是否每秒打印一次调试信息
static constexpr bool DEBUG_SENSOR_PRINT = true;

// PID 控制器（左右轮各一套）
static PID pidL;
static PID pidR;

// 目标速度（cm/s），后续可通过串口指令修改
static float target_left_cm_s = 0.0f;
static float target_right_cm_s = 0.0f;

// 最近一次计算得到的状态（用于发送/打印）
static int32_t latest_left_count = 0;
static int32_t latest_right_count = 0;
static float latest_left_cm_s = 0.0f;
static float latest_right_cm_s = 0.0f;
static int latest_pwmL = 0;
static int latest_pwmR = 0;

extern float roll, pitch, yaw;
extern int16_t ax, ay, az;
extern int16_t gx, gy, gz;

// 用 10ms tick 
static Ticker ticker;

// 由定时回调置位，在 loop() 里读取并清零
// 必须用 volatile，避免编译器优化导致读不到置位结果
static volatile bool due_50ms = false;
static volatile bool due_100ms = false;
static volatile bool due_1s = false;
static void on_tick_10ms();

void setup()
{
  Serial.begin(115200);

  // 底盘初始化
  wheel_encoder_init();
  wheel_encoder_speed_init();
  motor_init();
  if(!mpu6050_init()) {
    Serial.println("MPU6050 初始化失败");
    while(true);
  }
  mpu6050_calibrate();
  // PID 参数初始化
  pidL = {};
  pidR = {};
  PID_Init(&pidL, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);
  PID_Init(&pidR, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);

  target_left_cm_s = 0.0f;
  target_right_cm_s = 0.0f;

  // 单实例定时器：10ms tick
  ticker.attach_ms(10, on_tick_10ms);
}

void loop()
{
  // 1) 串口接收：必须放在 loop() 最前面，保证接收及时
  if (Serial.available() > 0) {
    char c = Serial.read();
    // 后续在此处添加指令解析逻辑
  }

  // 2) 每 50ms 执行一次：PID
  if (due_50ms) {
    due_50ms = false;

    int32_t left_count = 0;
    int32_t right_count = 0;
    float left_cm_s = 0.0f;
    float right_cm_s = 0.0f;

    // 读取编码器：总计数 + 速度（cm/s）
    wheel_encoder_get_counts(&left_count, &right_count);
    wheel_encoder_get_speed_cm_s(&left_cm_s, &right_cm_s,50000U);
    mpu6050_update();
    latest_left_count = left_count;
    latest_right_count = right_count;
    latest_left_cm_s = left_cm_s;
    latest_right_cm_s = right_cm_s;

    // 目标为 0 时清一次 PID 累积，避免停止后再次启动抖动
    if (target_left_cm_s == 0) PID_Clear(&pidL);
    if (target_right_cm_s == 0) PID_Clear(&pidR);

    // PID 计算并输出到电机
    float pwmL_float = PID_IncPIDCal(&pidL, left_cm_s, target_left_cm_s);
    float pwmR_float = PID_IncPIDCal(&pidR, right_cm_s, target_right_cm_s);
    int pwmL = constrain((int)pwmL_float, -255, 255);
    int pwmR = constrain((int)pwmR_float, -255, 255);
    motorA_set(pwmL);
    motorB_set(pwmR);

    latest_pwmL = pwmL;
    latest_pwmR = pwmR;
  }
  // 3) 每 100ms 执行一次：发送传感器数据
  if (due_100ms) {
    due_100ms = false;
    uint8_t enc_payload[6];
    uint8_t imu_payload[18];
    ss_pack_int_le(latest_left_count, 3, &enc_payload[0]);//打包左轮总计数
    ss_pack_int_le(latest_right_count, 3, &enc_payload[3]);//打包右轮总计数
    ss_pack_mpu6050_bundle(ax,ay,az,gx,gy,gz,roll,pitch,yaw,imu_payload);//打包IMU数据帧
    ss_send(Serial, FRAME_FUNC_ENCODER, enc_payload, 6, false);//发送编码器总计数
    ss_send(Serial, FRAME_FUNC_IMU, imu_payload, 18, false);//发送IMU数据帧
  }



  // 3) 每 1 秒打印一次调试信息
  if (due_1s && DEBUG_SENSOR_PRINT) {
    due_1s = false;

    Serial.print("[ENC] countL=");
    Serial.print(latest_left_count);
    Serial.print(" countR=");
    Serial.print(latest_right_count);
    Serial.print(" vL_cm_s=");
    Serial.print(latest_left_cm_s);
    Serial.print(" vR_cm_s=");
    Serial.print(latest_right_cm_s);
    Serial.print(" pwmL=");
    Serial.print(latest_pwmL);
    Serial.print(" pwmR=");
    Serial.print(latest_pwmR);
    Serial.print(" roll=");
    Serial.print(roll);
    Serial.print(" pitch=");
    Serial.print(pitch);
    Serial.print(" yaw=");
    Serial.println(yaw);
  }
}

// 每 10ms 触发一次：用计数器合成 100ms 与 1s 的“到期标记”
static void on_tick_10ms()
{
  static uint16_t c50ms = 0;
  static uint16_t c100ms = 0;
  static uint16_t c1s = 0;

  c50ms++;
  c100ms++;
  c1s++;

  if (c50ms >= 5) {
    c50ms = 0;
    due_50ms = true;
  }

  if (c100ms >= 10) {
    c100ms = 0;
    due_100ms = true;
  }

  if (c1s >= 100) {
    c1s = 0;
    due_1s = true;
  }
}
