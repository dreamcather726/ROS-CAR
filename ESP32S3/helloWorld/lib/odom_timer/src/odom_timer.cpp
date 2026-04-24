#include "odom_timer.h"

static hw_timer_t *g_timer = nullptr;
static portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;
static volatile bool g_due = false;

static void IRAM_ATTR on_timer()
{
  portENTER_CRITICAL_ISR(&g_mux);
  g_due = true;
  portEXIT_CRITICAL_ISR(&g_mux);
}

void odom_timer_init(uint32_t period_us)
{
  if (g_timer) {
    timerAlarmDisable(g_timer);
    timerDetachInterrupt(g_timer);
    timerEnd(g_timer);
    g_timer = nullptr;
  }

  portENTER_CRITICAL(&g_mux);
  g_due = false;
  portEXIT_CRITICAL(&g_mux);

  g_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(g_timer, &on_timer, true);
  timerAlarmWrite(g_timer, period_us, true);
  timerAlarmEnable(g_timer);
}

bool odom_timer_due()
{
  bool due = false;
  portENTER_CRITICAL(&g_mux);
  if (g_due) {
    g_due = false;
    due = true;
  }
  portEXIT_CRITICAL(&g_mux);
  return due;
}
