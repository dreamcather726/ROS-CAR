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

    long duration = pulseIn(ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT_US);

    // 算距离：声速 340m/s，往返除以2
    float distance = (duration * 0.034) / 2.0;

    // 超出范围或无信号返回 -1
    if (distance == 0 || distance > 400) {
        return -1;
    }

    return distance;
}
    
static inline void write_i16_le(uint8_t *out, int16_t v)
{
    out[0] = static_cast<uint8_t>(v & 0xFF);
    out[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

void ultrasonic_pack_distance_cm_x10(float distance_cm, uint8_t out6[6])
{
    int32_t dv = -1;
    if (distance_cm >= 0.0f) {
        dv = static_cast<int32_t>(distance_cm * 10.0f);
        if (dv > 32767) dv = 32767;
        if (dv < -32768) dv = -32768;
    }

    out6[2] = 0;
    out6[3] = 0;
    out6[4] = 0;
    out6[5] = 0;
    write_i16_le(&out6[0], static_cast<int16_t>(dv));
}
