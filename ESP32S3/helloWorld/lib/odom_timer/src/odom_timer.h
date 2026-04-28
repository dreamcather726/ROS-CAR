#pragma once

#include <Arduino.h>

typedef struct
{
  uint8_t timer_num;
} Timer;

void timer_init(Timer *t, uint8_t timer_num, uint32_t period_us);
bool timer_due(Timer *t);

void odom_timer_init(uint32_t period_us);
bool odom_timer_due();
