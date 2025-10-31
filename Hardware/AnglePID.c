#include "stm32f10x.h"                  // Device header
#include "AnglePID.h"

/**
  * 角度PID初始化（适配角度跟随，精度优先）
  */
void AnglePID_Init(AnglePID_HandleTypeDef *pid)
{
    pid->Kp = 12.0f;   // 比例系数（比速度PID大，提升响应）
    pid->Ki = 0.3f;    // 积分系数（小值消除静差，避免震荡）
    pid->Kd = 2.5f;    // 微分系数（抑制跟随震荡）
    pid->Target = 0;   // 初始目标角度=0（电机1初始位置）
    pid->Feedback = 0;
    pid->Err = 0;
    pid->ErrLast = 0;
    pid->ErrPrev = 0;
    pid->Output = 0;
    pid->OutMax = 700; // 与电机PWM范围一致
    pid->OutMin = -700;
}

/**
  * 增量式PID计算（角度专用，累计误差优化）
  */
int16_t AnglePID_Calculate(AnglePID_HandleTypeDef *pid)
{
    pid->Err = pid->Target - pid->Feedback;
    
    // 增量式公式（优化积分项，避免累计溢出）
    int16_t deltaU = pid->Kp * (pid->Err - pid->ErrLast)
                   + pid->Ki * pid->Err
                   + pid->Kd * (pid->Err - 2 * pid->ErrLast + pid->ErrPrev);
    
    pid->Output += deltaU;
    
    // 输出限幅
    if (pid->Output > pid->OutMax) pid->Output = pid->OutMax;
    if (pid->Output < pid->OutMin) pid->Output = pid->OutMin;
    
    // 更新误差历史
    pid->ErrPrev = pid->ErrLast;
    pid->ErrLast = pid->Err;
    
    return pid->Output;
}

void AnglePID_SetTarget(AnglePID_HandleTypeDef *pid, int32_t target)
{
    pid->Target = target;
}

void AnglePID_UpdateFeedback(AnglePID_HandleTypeDef *pid, int32_t feedback)
{
    pid->Feedback = feedback;
}
