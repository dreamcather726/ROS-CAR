#include "speed_pid.h"

// PID初始化（只设置参数，不清零状态）
void PID_Init(PID *pid, float Kp, float Ki, float Kd, float max, float min)
{
    // 1. 检查参数有效性（防止除0）
    if(Kp == 0.0f || Ki == 0.0f || Kd == 0.0f || max == min) return;

    // 2. 保存参数（不改变 Ki）
    pid->P = Kp;
    pid->I = Ki;
    pid->D = Kd;
    pid->OutputMax = max;
    pid->OutputMin = min;

}

// 重置PID状态（不清空参数）
void PID_Clear(PID *pid)
{
    pid->LastError = 0.0f;
    pid->PrevError = 0.0f;
    pid->Output = 0.0f;
}

// 增量式PID计算（电机速度专用）
float PID_IncPIDCal(PID *pid, float NowValue, float AimValue)
{
    float iError;      // 当前误差
    float increment;   // 增量值

    // 1. 计算误差
    iError = AimValue - NowValue;

    // 2. 增量式PID公式（标准工业版）
    increment = pid->P * (iError - pid->LastError)                         // P
              + pid->I * iError                                            // I
              + pid->D * (iError - 2 * pid->LastError + pid->PrevError);   // D

    // 3. 累加增量
    pid->Output += increment;

    // 4. 输出限幅（防止电机过载）
    if(pid->Output > pid->OutputMax) pid->Output = pid->OutputMax;
    if(pid->Output < pid->OutputMin) pid->Output = pid->OutputMin;

    // 5. 更新历史误差
    pid->PrevError = pid->LastError;
    pid->LastError = iError;

    return pid->Output;
}

//速度环PID参数（ P=0.5, I=1.4, D=0.2）