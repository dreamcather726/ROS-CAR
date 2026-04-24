#include "ultrasonic.h"

void ultrasonic_init() {
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    digitalWrite(TRIG_PIN, LOW);
}

float ultrasonic_get_distance() {
    // 发触发信号
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);

    // 读回响信号（超时 30ms = 约 5 米）
    long duration = pulseIn(ECHO_PIN, HIGH, 30000);

    // 算距离：声速 340m/s，往返除以2
    float distance = (duration * 0.034) / 2.0;

    // 超出范围或无信号返回 -1
    if (distance == 0 || distance > 400) {
        return -1;
    }

    return distance;
}