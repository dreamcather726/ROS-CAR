#pragma once

#include <Arduino.h>

static constexpr uint8_t MOTOR_PWMA_PIN = 1;
static constexpr uint8_t MOTOR_DIRA_PIN = 2;
static constexpr uint8_t MOTOR_PWMB_PIN = 4;
static constexpr uint8_t MOTOR_DIRB_PIN = 5;

static constexpr bool MOTOR_A_INVERT_DIR = false;
static constexpr bool MOTOR_B_INVERT_DIR = true;

void motor_init();
void motorA_set(int speed);
void motorB_set(int speed);
void motorA_stop();
void motorB_stop();
void motor_stop();
