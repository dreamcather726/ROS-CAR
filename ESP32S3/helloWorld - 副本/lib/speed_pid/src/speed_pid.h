#include <Arduino.h>

typedef struct
{
    // PID参数
    float P;
    float I;
    float D;

    // 输出限幅
    float OutputMax;
    float OutputMin;

    // 历史误差
    float LastError;
    float PrevError;

    // 输出值
    float Output;
} PID;

// 函数声明
void PID_Init(PID *pid,float Kp,float Ki,float Kd,float max,float min);
void PID_Clear(PID *pid);
float PID_IncPIDCal(PID *pid, float NowValue, float AimValue);
