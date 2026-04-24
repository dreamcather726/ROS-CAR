#pragma once

#include <Arduino.h>

void odom_timer_init(uint32_t period_us);
bool odom_timer_due();
