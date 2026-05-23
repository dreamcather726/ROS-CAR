#pragma once

#include <Arduino.h>

typedef struct {
  uint32_t period_ms;
  uint32_t last_ms;
} SimpleTimer;

void simple_timer_init(SimpleTimer *timer, uint32_t period_ms);
bool simple_timer_due(SimpleTimer *timer);
void simple_timer_wait_us(uint32_t wait_us);
