#include "stm32f10x.h"                  // Device header

#include "PID.h"

/**
  * @brief  PID初始化（可根据电机特性调整Kp/Ki/Kd/OutMax）
  * @param  pid: PID结构体指针
  */
void PID_Init(PID_HandleTypeDef *pid)
{
    pid->Kp = 8.0f;    // 比例系数（核心，越大响应越快，过大会超调）
    pid->Ki = 0.2f;    // 积分系数（消除静差，过大会超调）
    pid->Kd = 1.5f;    // 微分系数（抑制超调，过大响应变慢）
    pid->Target = 0;   // 初始目标速度=0（回正功能）
    pid->Feedback = 0;
    pid->Err = 0;
    pid->ErrLast = 0;
    pid->ErrPrev = 0;
    pid->Output = 0;
    pid->OutMax = 700; // PWM最大输出（根据电机调整，建议0~720）
    pid->OutMin = -700;// PWM最小输出（负向反转）
}

/**
  * @brief  增量式PID计算
  * @param  pid: PID结构体指针
  * @return PID输出值（用于控制PWM）
  */
int16_t PID_Calculate(PID_HandleTypeDef *pid)
{
    // 计算当前误差（目标-反馈）
    pid->Err = pid->Target - pid->Feedback;
    
    // 增量式PID公式：Δu = Kp(Err - ErrLast) + Ki*Err + Kd(Err - 2ErrLast + ErrPrev)
    int16_t deltaU = pid->Kp * (pid->Err - pid->ErrLast) 
                   + pid->Ki * pid->Err 
                   + pid->Kd * (pid->Err - 2 * pid->ErrLast + pid->ErrPrev);
    
    // 更新输出（累加增量）
    pid->Output += deltaU;
    
    // 输出限幅（防止PWM溢出，保护电机）
    if (pid->Output > pid->OutMax) pid->Output = pid->OutMax;
    if (pid->Output < pid->OutMin) pid->Output = pid->OutMin;
    
    // 更新误差历史（用于下一次计算）
    pid->ErrPrev = pid->ErrLast;
    pid->ErrLast = pid->Err;
    
    return pid->Output;
}

/**
  * @brief  设置PID目标速度
  * @param  pid: PID结构体指针
  * @param  target: 目标速度（编码器增量值/10ms，正负代表方向）
  */
void PID_SetTarget(PID_HandleTypeDef *pid, int16_t target)
{
    pid->Target = target;
}

/**
  * @brief  更新PID反馈值（从编码器获取）
  * @param  pid: PID结构体指针
  * @param  feedback: 编码器实际增量值（10ms内）
  */
void PID_UpdateFeedback(PID_HandleTypeDef *pid, int16_t feedback)
{
    pid->Feedback = feedback;
}