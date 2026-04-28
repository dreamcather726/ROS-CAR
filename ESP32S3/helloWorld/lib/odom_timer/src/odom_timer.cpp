#include "odom_timer.h"

static hw_timer_t *g_timers[4] = {nullptr, nullptr, nullptr, nullptr};
static portMUX_TYPE g_muxes[4] = {
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
    portMUX_INITIALIZER_UNLOCKED,
};
static volatile bool g_due[4] = {false, false, false, false};

static void IRAM_ATTR on_timer_0()
{
  portENTER_CRITICAL_ISR(&g_muxes[0]);
  g_due[0] = true;
  portEXIT_CRITICAL_ISR(&g_muxes[0]);
}

static void IRAM_ATTR on_timer_1()
{
  portENTER_CRITICAL_ISR(&g_muxes[1]);
  g_due[1] = true;
  portEXIT_CRITICAL_ISR(&g_muxes[1]);
}

static void IRAM_ATTR on_timer_2()
{
  portENTER_CRITICAL_ISR(&g_muxes[2]);
  g_due[2] = true;
  portEXIT_CRITICAL_ISR(&g_muxes[2]);
}

static void IRAM_ATTR on_timer_3()
{
  portENTER_CRITICAL_ISR(&g_muxes[3]);
  g_due[3] = true;
  portEXIT_CRITICAL_ISR(&g_muxes[3]);
}

static void (*const g_isrs[4])() = {on_timer_0, on_timer_1, on_timer_2, on_timer_3};

void timer_init(Timer *t, uint8_t timer_num, uint32_t period_us)
{
  if (!t) return;
  if (timer_num > 3) timer_num = 3;
  t->timer_num = timer_num;

  if (g_timers[timer_num]) {
    timerAlarmDisable(g_timers[timer_num]);
    timerDetachInterrupt(g_timers[timer_num]);
    timerEnd(g_timers[timer_num]);
    g_timers[timer_num] = nullptr;
  }

  portENTER_CRITICAL(&g_muxes[timer_num]);
  g_due[timer_num] = false;
  portEXIT_CRITICAL(&g_muxes[timer_num]);

  g_timers[timer_num] = timerBegin(timer_num, 80, true);
  timerAttachInterrupt(g_timers[timer_num], g_isrs[timer_num], true);
  timerAlarmWrite(g_timers[timer_num], period_us, true);
  timerAlarmEnable(g_timers[timer_num]);
}

bool timer_due(Timer *t)
{
  if (!t) return false;
  const uint8_t timer_num = t->timer_num > 3 ? 3 : t->timer_num;

  bool due = false;
  portENTER_CRITICAL(&g_muxes[timer_num]);
  if (g_due[timer_num]) {
    g_due[timer_num] = false;
    due = true;
  }
  portEXIT_CRITICAL(&g_muxes[timer_num]);
  return due;
}

void odom_timer_init(uint32_t period_us)
{
  static Timer t0 = {0};
  timer_init(&t0, 0, period_us);
}

bool odom_timer_due()
{
  static Timer t0 = {0};
  return timer_due(&t0);
}
