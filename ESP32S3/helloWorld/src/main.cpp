#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include <serial_sender.h>
#include <odom_timer.h>
#include <ultrasonic.h>
#include <odom_frame.h>

#define SERIAL_BAUD 115200

static constexpr uint8_t FUNC_ODOM_COUNTS = 0x10;
static constexpr uint8_t FUNC_ODOM_SPEED = 0x11;
static constexpr uint8_t FUNC_ODOM_DISTANCE = 0x12;//超声波数据帧大小

static constexpr uint32_t ODOM_PERIOD_US = 100000U;// 100ms
static float last_distance = 0.0f;

void setup() {
  Serial.begin(SERIAL_BAUD);
  ultrasonic_init();
  wheel_encoder_init();
  wheel_encoder_speed_init();
  motor_init();
  odom_timer_init(ODOM_PERIOD_US);
}

void loop() {
  motorA_set(128);
  motorB_set(128);

  if (!odom_timer_due()) return;

  int32_t left_count = 0;
  int32_t right_count = 0;
  // 获取超声波距离
  float distance = ultrasonic_get_distance();

  float left_cm_s = 0.0f;// 速度单位：cm/s * 1000
  float right_cm_s = 0.0f;// 速度单位：cm/s * 1000
  if (!wheel_encoder_get_odom(&left_count, &right_count, &left_cm_s, &right_cm_s, ODOM_PERIOD_US)) return;// 获取里程计数据

  if (Serial.availableForWrite() < static_cast<int>(SERIAL_SENDER_FRAME_SIZE * 2)) return;// 检查是否有足够的空间发送数据

  uint8_t data_counts[SERIAL_SENDER_DATA_SIZE];// 里程计数据
  uint8_t data_speed[SERIAL_SENDER_DATA_SIZE];// 速度数据
  if (!odom_frame_build_counts_i24_le(left_count, right_count, data_counts)) return;// 构建里程计数据
  if (!odom_frame_build_speed_i16_le_x100(left_cm_s, right_cm_s, data_speed)) return;// 构建速度数据
  ss_send(Serial, FUNC_ODOM_COUNTS, data_counts, false);// 发送里程计数据
  ss_send(Serial, FUNC_ODOM_SPEED, data_speed, false);// 发送速度数据
  ss_send(Serial, FUNC_ODOM_DISTANCE, &distance, sizeof(distance));// 发送超声波距离数据
  
  //打印原始数据
  // Serial.println(distance);// 打印距离
  // Serial.println("ODOM:");
  // Serial.println("Left count: ");
  // Serial.println(left_count);// 打印左轮里程计数据
  // Serial.println("Right count: ");  
  // Serial.println(right_count);// 打印右轮里程计数据
  // Serial.println("Left speed: ");
  // Serial.println(left_cm_s);// 打印左轮速度
  // Serial.println("Right speed: ");  
  // Serial.println(right_cm_s);// 打印右轮速度
}
