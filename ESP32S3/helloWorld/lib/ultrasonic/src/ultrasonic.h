#pragma once

#include <stdint.h>
#include <Arduino.h>

#define TRIG_PIN     39
#define ECHO_PIN     40

void ultrasonic_init(void);
float ultrasonic_get_distance(void);