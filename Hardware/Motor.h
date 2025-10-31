#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

void Motor_Init(void);          // 电机和PWM初始化
void Motor_SetSpeed(int16_t speed);  // 设置电机速度（接收PID输出）

#endif
