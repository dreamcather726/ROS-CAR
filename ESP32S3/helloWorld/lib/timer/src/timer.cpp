#include "timer.h"

void simple_timer_init(SimpleTimer *timer, uint32_t period_ms) {
  if (timer == nullptr) {
    return;
  }

  timer->period_ms = period_ms;
  timer->last_ms = millis();
}

bool simple_timer_due(SimpleTimer *timer) {
  if (timer == nullptr) {
    return false;
  }

  uint32_t now_ms = millis();
  if (static_cast<uint32_t>(now_ms - timer->last_ms) < timer->period_ms) {
    return false;
  }

  timer->last_ms = now_ms;
  return true;
}

void simple_timer_wait_us(uint32_t wait_us) {
  uint32_t start_us = micros();
  while (static_cast<uint32_t>(micros() - start_us) < wait_us) {
  }
}
