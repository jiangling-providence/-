#include "Motor2.h"

/**
  * 电机2初始化（TIM8生成PWM，避免与电机1冲突）
  * 硬件连接：
  * PB0 → 电机2 PWM（L298N ENB/ENA）
  * PB13 → 方向1（IN3）
  * PB14 → 方向2（IN4）
  */
void Motor2_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_TIM8, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    
    // GPIO初始化
    GPIO_InitTypeDef GPIO_InitStruct;
    // PB0：复用推挽输出（TIM8_CH1）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStruct);
    // PB13/PB14：方向控制
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM8初始化（与电机1同频率PWM，保证一致性）
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = 71;
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 719;
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStruct);
    
    // PWM模式配置
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM8, &TIM_OCInitStruct);
    
    // 高级定时器主输出使能
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
    
    // 初始停转
    Motor2_SetSpeed(0);
}

/**
  * 设置电机2速度（与电机1逻辑一致）
  */
void Motor2_SetSpeed(int16_t speed)
{
    if (speed > 0)  // 正转
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        GPIO_ResetBits(GPIOA, GPIO_Pin_12);
        TIM_SetCompare1(TIM8, speed);
    }
    else if (speed < 0)  // 反转
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        GPIO_SetBits(GPIOA, GPIO_Pin_12);
        TIM_SetCompare1(TIM8, -speed);
    }
    else  // 停转
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_11 | GPIO_Pin_12);
        TIM_SetCompare1(TIM8, 0);
    }
}
