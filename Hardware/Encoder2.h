#ifndef __ENCODER2_H
#define __ENCODER2_H

#include "stm32f10x.h"

void Encoder2_Init(void);  // 电机2编码器初始化（PB6/PB7-TIM4）
int16_t Encoder2_Get(void); // 获取电机2编码器增量值（10ms内）

#endif
