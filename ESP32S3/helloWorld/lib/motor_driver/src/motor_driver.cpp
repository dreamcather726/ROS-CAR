#include <Arduino.h>
#include "motor_driver.h"



void motor_init()
{
  pinMode(MOTOR_DIRA_PIN, OUTPUT);
  pinMode(MOTOR_DIRB_PIN, OUTPUT);
  pinMode(MOTOR_PWMA_PIN, OUTPUT);
  pinMode(MOTOR_PWMB_PIN, OUTPUT);
  digitalWrite(MOTOR_DIRA_PIN, LOW);
  digitalWrite(MOTOR_DIRB_PIN, LOW);

  analogWrite(MOTOR_PWMA_PIN, 0);
  analogWrite(MOTOR_PWMB_PIN, 0);
}

void motorA_set(int speed)
{
  speed = constrain(speed, -255, 255);
  const bool forward = (speed >= 0);
  digitalWrite(MOTOR_DIRA_PIN, forward ? HIGH : LOW);

  analogWrite(MOTOR_PWMA_PIN, abs(speed));
  delay(1);
}

void motorB_set(int speed)
{
  speed = constrain(speed, -255, 255);
  const bool forward = (speed >= 0);
  digitalWrite(MOTOR_DIRB_PIN, forward ? HIGH : LOW);
 
  analogWrite(MOTOR_PWMB_PIN, abs(speed));
  delay(1);
}

void motorA_stop()
{
  analogWrite(MOTOR_PWMA_PIN, 0);
}

void motorB_stop()
{
  analogWrite(MOTOR_PWMB_PIN, 0);
}

void motor_stop()
{
  motorA_stop();
  motorB_stop();
}
