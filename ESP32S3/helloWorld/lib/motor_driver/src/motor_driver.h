

#include <Arduino.h>

static constexpr uint8_t MOTOR_PWMA_PIN = 6;
static constexpr uint8_t MOTOR_DIRA_PIN = 7;
static constexpr uint8_t MOTOR_PWMB_PIN = 15;
static constexpr uint8_t MOTOR_DIRB_PIN = 16;

void motor_init();
void motorA_set(int speed);
void motorB_set(int speed);
void motorA_stop();
void motorB_stop();
void motor_stop();
