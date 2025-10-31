#ifndef __ANGLE_PID_H
#define __ANGLE_PID_H

// 角度PID结构体（适配双电机角度跟随）
typedef struct {
    float Kp;         // 比例系数（核心，影响响应速度）
    float Ki;         // 积分系数（消除静差，提升精度）
    float Kd;         // 微分系数（抑制震荡）
    int32_t Target;   // 目标角度（电机1累计编码器值）
    int32_t Feedback; // 反馈角度（电机2累计编码器值）
    int32_t Err;      // 当前误差
    int32_t ErrLast;  // 上一次误差
    int32_t ErrPrev;  // 上上次误差
    int16_t Output;   // PID输出（电机2 PWM控制）
    int16_t OutMax;   // 输出限幅（与速度PID一致）
    int16_t OutMin;   // 负向限幅
} AnglePID_HandleTypeDef;

// 函数声明
void AnglePID_Init(AnglePID_HandleTypeDef *pid);          // 初始化
int16_t AnglePID_Calculate(AnglePID_HandleTypeDef *pid);  // 计算输出
void AnglePID_SetTarget(AnglePID_HandleTypeDef *pid, int32_t target);  // 设置目标角度
void AnglePID_UpdateFeedback(AnglePID_HandleTypeDef *pid, int32_t feedback);  // 更新反馈

#endif
