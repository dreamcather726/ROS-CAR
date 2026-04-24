
#include <Arduino.h>

// 引脚定义（你可以根据硬件修改）
#define TRIG_PIN    47
#define ECHO_PIN    48

// 初始化超声波
void ultrasonic_init();

// 获取距离，单位：厘米 cm
// 超时返回 -1
float ultrasonic_get_distance();
        
void ultrasonic_pack_distance_cm_x10(float distance_cm, uint8_t out6[6]);
