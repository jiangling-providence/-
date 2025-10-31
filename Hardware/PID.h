#ifndef __PID_H
#define __PID_H


// PID参数结构体（可在PID_Init中调整）
typedef struct {
    float Kp;         // 比例系数
    float Ki;         // 积分系数
    float Kd;         // 微分系数
    int16_t Target;   // 目标速度（编码器增量值/10ms）
    int16_t Feedback; // 反馈速度（编码器实际增量值/10ms）
    int16_t Err;      // 当前误差
    int16_t ErrLast;  // 上一次误差
    int16_t ErrPrev;  // 上上次误差
    int16_t Output;   // PID输出（PWM占空比相关）
    int16_t OutMax;   // 输出最大值（限制PWM范围）
    int16_t OutMin;   // 输出最小值（负向最大值）
} PID_HandleTypeDef;

// 函数声明
void PID_Init(PID_HandleTypeDef *pid);          // PID初始化（配置参数）
int16_t PID_Calculate(PID_HandleTypeDef *pid);  // PID计算（增量式）
void PID_SetTarget(PID_HandleTypeDef *pid, int16_t target);  // 设置目标速度
void PID_UpdateFeedback(PID_HandleTypeDef *pid, int16_t feedback);  // 更新反馈值

#endif
