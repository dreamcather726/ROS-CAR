#include "button.h"

static inline uint8_t normalize_read(uint8_t raw, uint8_t active_low)
{
  if (active_low) {
    return raw == LOW ? 1U : 0U;
  }
  return raw == HIGH ? 1U : 0U;
}

void button_init(Button *b, uint8_t pin, bool pullup, bool active_low, uint32_t debounce_ms)
{
  if (!b) return;
  b->pin = pin;
  b->active_low = active_low ? 1U : 0U;
  b->debounce_ms = debounce_ms;
  b->last_change_ms = millis();

  pinMode(pin, pullup ? INPUT_PULLUP : INPUT);
  const uint8_t raw = static_cast<uint8_t>(digitalRead(pin));
  const uint8_t pressed = normalize_read(raw, b->active_low);
  b->stable = pressed;
  b->last_read = pressed;
}

bool button_update_released(Button *b)
{
  if (!b) return false;

  const uint32_t now_ms = millis();
  const uint8_t raw = static_cast<uint8_t>(digitalRead(b->pin));
  const uint8_t pressed = normalize_read(raw, b->active_low);

  if (pressed != b->last_read) {
    b->last_read = pressed;
    b->last_change_ms = now_ms;
  }

  if ((now_ms - b->last_change_ms) < b->debounce_ms) {
    return false;
  }

  if (pressed == b->stable) {
    return false;
  }

  const uint8_t prev_stable = b->stable;
  b->stable = pressed;

  return (prev_stable == 1U) && (b->stable == 0U);
}

bool button_update_pressed(Button *b)
{
  if (!b) return false;

  const uint32_t now_ms = millis();
  const uint8_t raw = static_cast<uint8_t>(digitalRead(b->pin));
  const uint8_t pressed = normalize_read(raw, b->active_low);

  if (pressed != b->last_read) {
    b->last_read = pressed;
    b->last_change_ms = now_ms;
  }

  if ((now_ms - b->last_change_ms) < b->debounce_ms) {
    return false;
  }

  if (pressed == b->stable) {
    return false;
  }

  const uint8_t prev_stable = b->stable;
  b->stable = pressed;

  return (prev_stable == 0U) && (b->stable == 1U);
}

bool button_is_pressed(const Button *b)
{
  if (!b) return false;
  return b->stable == 1U;
}
