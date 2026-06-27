#include <Arduino.h>
#include "motor_driver.h"

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
  pwm = constrain(pwm, -255, 255);
  bool forward = (pwm >= 0);
  if (RIGHT_MOTOR_INVERT_DIR) forward = !forward;
  digitalWrite(RIGHT_MOTOR_DIR_PIN, forward ? HIGH : LOW);

  analogWrite(RIGHT_MOTOR_PWM_PIN, abs(pwm));

}

void set_left_motor_pwm(int pwm)
{
  pwm = constrain(pwm, -255, 255);
  bool forward = (pwm >= 0);
  if (LEFT_MOTOR_INVERT_DIR) forward = !forward;
  digitalWrite(LEFT_MOTOR_DIR_PIN, forward ? HIGH : LOW);
  analogWrite(LEFT_MOTOR_PWM_PIN, abs(pwm));
}
