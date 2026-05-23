#include <Arduino.h>
#include <motor_driver.h>
#include <wheel_encoder.h>
#include "speed_pid.h"
#include "serial_sender.h"
#include "timer.h"
#include "ultrasonic.h"
#include "mpu6050.h"
#include "Ticker.h"

// 功能码定义
static constexpr uint8_t FRAME_FUNC_ENCODER = 0x01;
static constexpr uint8_t FRAME_FUNC_MPU6050 = 0x02;
static constexpr uint8_t FRAME_FUNC_ULTRASONIC = 0x03;
static constexpr bool DEBUG_PID_PRINT = false;// 是否打印 PID 计算结果
static constexpr bool DEBUG_SENSOR_PRINT = true;// 是否打印传感器数据

static constexpr uint32_t PID_PERIOD_MS = 100U;// PID 周期 100ms
static constexpr uint32_t SENSOR_TX_PERIOD_MS = 100U;// 传感器数据发送周期 100ms
static constexpr uint32_t SENSOR_RX_PERIOD_MS = 50U;// 传感器数据接收周期 10ms

static SimpleTimer pid_timer = {};
static SimpleTimer sensor_tx_timer = {};
static SimpleTimer sensor_rx_timer = {};


static PID pidL;
static PID pidR;
static float target_left_cm_s = 0.0f;
static float target_right_cm_s = 0.0f;
static float ultrasonic_distance = 0.0f;
static int32_t latest_left_count = 0;
static int32_t latest_right_count = 0;
void setup() {
  // 必须先开串口！ 

  Serial.begin(115200);
  delay(100);
  wheel_encoder_init();
  wheel_encoder_speed_init();
  motor_init();
  ultrasonic_init();
  simple_timer_init(&pid_timer, PID_PERIOD_MS);
  simple_timer_init(&sensor_tx_timer, SENSOR_TX_PERIOD_MS);
  simple_timer_init(&sensor_rx_timer, SENSOR_RX_PERIOD_MS);

  pidL = {};
  pidR = {};
  PID_Init(&pidL, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);
  PID_Init(&pidR, 1.2f, 0.55f, 0.0f, 255.0f, -255.0f);

  target_left_cm_s = 0.00f;
  target_right_cm_s = 0.0f;
}

void loop() {

   if (simple_timer_due(&sensor_rx_timer)) {
    
    ultrasonic_distance = ultrasonic_get_distance();
  }
  // ====================== 100ms PID 调速 ======================
  if (simple_timer_due(&pid_timer)) {// PID 周期到
    int32_t left_count = 0;
    int32_t right_count = 0;
    float left_cm_s = 0.0f;
    float right_cm_s = 0.0f;

    wheel_encoder_get_counts(&left_count, &right_count);
    wheel_encoder_get_speed_cm_s(&left_cm_s, &right_cm_s, 0U);
    latest_left_count = left_count;
    latest_right_count = right_count;

    // PID 计算
    float pwmL_float = PID_IncPIDCal(&pidL, left_cm_s, target_left_cm_s);
    float pwmR_float = PID_IncPIDCal(&pidR, right_cm_s, target_right_cm_s);
    int pwmL = constrain((int)pwmL_float, -255, 255);
    int pwmR = constrain((int)pwmR_float, -255, 255);

    motorA_set(pwmL);
    motorB_set(pwmR);

    if (DEBUG_PID_PRINT) {
      Serial.print(target_left_cm_s);
      Serial.print(",");
      Serial.print(left_cm_s);
      Serial.print(",");
      Serial.println(pwmL);
    }
  }
   if (simple_timer_due(&sensor_tx_timer)) {// 传感器数据发送周期到MUSE PI
    ss_send_ultrasonic_distance_cm_x10(Serial,FRAME_FUNC_ULTRASONIC,ultrasonic_distance,false);
    ss_send_encoder_counts(Serial, FRAME_FUNC_ENCODER, latest_left_count, latest_right_count);
  }
  static SimpleTimer debug_timer = {};
  if (DEBUG_SENSOR_PRINT && simple_timer_due(&debug_timer)) {
    simple_timer_init(&debug_timer, 1000);  // 1秒打印一次
    Serial.print("[Ultrasonic] ");
    Serial.print(ultrasonic_distance);
    Serial.println(" cm");
    
  }
  }

 


