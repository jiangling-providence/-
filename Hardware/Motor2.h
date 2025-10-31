#ifndef __MOTOR2_H
#define __MOTOR2_H

#include "stm32f10x.h"

void Motor2_Init(void);         // 电机2初始化（PWM+方向）
void Motor2_SetSpeed(int16_t speed);  // 设置电机2速度

#endif
