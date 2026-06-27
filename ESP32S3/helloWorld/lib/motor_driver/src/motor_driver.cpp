#include <Arduino.h>
#include "motor_driver.h"

static void set_motor_pwm_with_dead_time(uint8_t dir_pin,
                                         uint8_t pwm_pin,
                                         int pwm,
                                         bool invert_dir,
                                         bool *last_forward,
                                         bool *has_last_forward);

void motor_init()
{
  pinMode(RIGHT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_DIR_PIN, OUTPUT);
  pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
  pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
  digitalWrite(RIGHT_MOTOR_DIR_PIN, LOW);
  digitalWrite(LEFT_MOTOR_DIR_PIN, LOW);

  analogWrite(RIGHT_MOTOR_PWM_PIN, 0);
  analogWrite(LEFT_MOTOR_PWM_PIN, 0);
}

void set_right_motor_pwm(int pwm)
{
  static bool last_forward = true;
  static bool has_last_forward = false;

  set_motor_pwm_with_dead_time(RIGHT_MOTOR_DIR_PIN,
                               RIGHT_MOTOR_PWM_PIN,
                               pwm,
                               RIGHT_MOTOR_INVERT_DIR,
                               &last_forward,
                               &has_last_forward);
}

void set_left_motor_pwm(int pwm)
{
  static bool last_forward = true;
  static bool has_last_forward = false;
  set_motor_pwm_with_dead_time(LEFT_MOTOR_DIR_PIN,
                               LEFT_MOTOR_PWM_PIN,
                               pwm,
                               LEFT_MOTOR_INVERT_DIR,
                               &last_forward,
                               &has_last_forward);
}

static void set_motor_pwm_with_dead_time(uint8_t dir_pin,
                                         uint8_t pwm_pin,
                                         int pwm,
                                         bool invert_dir,
                                         bool *last_forward,
                                         bool *has_last_forward)
{
  pwm = constrain(pwm, -255, 255);
  bool forward = (pwm >= 0);
  if (invert_dir) forward = !forward;

  if (*has_last_forward && forward != *last_forward) {
    analogWrite(pwm_pin, 0);
    delayMicroseconds(MOTOR_DIRECTION_SWITCH_DELAY_US);
  }

  digitalWrite(dir_pin, forward ? HIGH : LOW);
  analogWrite(pwm_pin, abs(pwm));
  *last_forward = forward;
  *has_last_forward = true;
}
