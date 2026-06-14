#pragma once

#include <Arduino.h>

static constexpr uint8_t RIGHT_MOTOR_PWM_PIN = 4; // Driver channel A: PWMA
static constexpr uint8_t RIGHT_MOTOR_DIR_PIN = 5; // Driver channel A: AN1
static constexpr uint8_t LEFT_MOTOR_PWM_PIN = 1;  // Driver channel B: PWMB
static constexpr uint8_t LEFT_MOTOR_DIR_PIN = 2;  // Driver channel B: BN1

static constexpr bool RIGHT_MOTOR_INVERT_DIR = false;
static constexpr bool LEFT_MOTOR_INVERT_DIR = true;
static constexpr bool RIGHT_MOTOR_REVERSE_USE_FORWARD_OUTPUT = true;
static constexpr uint32_t MOTOR_DIRECTION_SWITCH_DELAY_US = 1000U;

void motor_init();
void set_right_motor_pwm(int pwm);
void set_left_motor_pwm(int pwm);
