#pragma once

#include <Arduino.h>

typedef struct
{
  uint8_t pin;
  uint8_t active_low;
  uint8_t stable;
  uint8_t last_read;
  uint32_t last_change_ms;
  uint32_t debounce_ms;
} Button;
/**
 * @brief 初始化按钮
 * 
 * @param b 按钮结构体指针
 * @param pin 按钮引脚
 * @param pullup 是否启用上拉电阻
 * @param active_low 是否为低电平有效
 * @param debounce_ms 去抖时间（毫秒）
 */
void button_init(Button *b, uint8_t pin, bool pullup, bool active_low, uint32_t debounce_ms);// 初始化按钮
bool button_update_pressed(Button *b);
bool button_update_released(Button *b);// 更新按钮状态
bool button_is_pressed(const Button *b);// 判断按钮是否被按下
