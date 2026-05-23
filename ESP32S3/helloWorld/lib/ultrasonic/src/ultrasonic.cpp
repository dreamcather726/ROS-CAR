#include "ultrasonic.h"
#include "timer.h"
#include <math.h>

// 宏定义统一管理（避免魔术数字）
#define TRIGGER_PULL_LOW_US    5      // 标准时序：触发前拉低5us
#define TRIGGER_HIGH_US        10     // 触发信号10us
#define SOUND_SPEED_CM_PER_US  0.034f
#define MAX_DISTANCE_CM        400.0f
#define DEFAULT_INVALID_VALUE  -1

// 滤波：采样次数
#define FILTER_SAMPLE_COUNT    5

void ultrasonic_init() {
    // 先置低 → 再初始化，防止误触发
    digitalWrite(TRIG_PIN, LOW);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
}

// 单次读取（内部函数）
static float ultrasonic_get_single(void) {
    digitalWrite(TRIG_PIN, LOW);
    simple_timer_wait_us(TRIGGER_PULL_LOW_US);
    
    digitalWrite(TRIG_PIN, HIGH);
    simple_timer_wait_us(TRIGGER_HIGH_US);
    digitalWrite(TRIG_PIN, LOW);

    const long timeout = (long)(MAX_DISTANCE_CM * 2 / SOUND_SPEED_CM_PER_US);
    long duration = pulseIn(ECHO_PIN, HIGH, timeout);

    if (duration <= 0) {
        return DEFAULT_INVALID_VALUE;
    }

    float distance = (duration * SOUND_SPEED_CM_PER_US) / 2.0f;

    if (distance <= 0.0f || distance > MAX_DISTANCE_CM) {
        return DEFAULT_INVALID_VALUE;
    }

    return distance;
}

// 多次采样取平均（无异常值剔除）
float ultrasonic_get_distance(void) {
    float sum = 0.0f;
    uint8_t count = 0;

    // 连续采样 N 次，累加有效数据
    for (int i = 0; i < FILTER_SAMPLE_COUNT; i++) {
        float d = ultrasonic_get_single();
        if (d >= 0.0f) {  // 只统计有效距离
            sum += d;
            count++;
        }
        simple_timer_wait_us(300); // 防止采样过快
    }

    // 无有效数据
    if (count == 0) {
        return DEFAULT_INVALID_VALUE;
    }

    // 返回平均值
    return sum / count;
}
