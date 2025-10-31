#include "stm32f10x.h"
#include "Motor.h"

/**
  * @brief  电机GPIO和PWM初始化（TIM1生成PWM，PA8=PWM1，PA11/PA12=方向控制）
  * 硬件连接：
  * PA8 → 电机PWM输入（L298N的ENA/ENB）
  * PA11 → 电机方向1（IN1）
  * PA12 → 电机方向2（IN2）
  */
void Motor_Init(void)
{
    // 开启时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE); // 复用功能时钟
    
    // GPIO初始化（PA8=PWM输出，PA11/PA12=普通输出）
    GPIO_InitTypeDef GPIO_InitStruct;
    // PA8：复用推挽输出（TIM1_CH1）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    // PA11/PA12：推挽输出（方向控制）
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TIM1初始化（PWM模式，频率≈1kHz，分辨率720）
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStruct;
    TIM_TimeBaseStruct.TIM_Prescaler = 71;       // 72MHz/(71+1)=1MHz
    TIM_TimeBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseStruct.TIM_Period = 719;         // 1MHz/720≈1.38kHz（PWM频率）
    TIM_TimeBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStruct);
    
    // PWM模式配置（TIM1_CH1）
    TIM_OCInitTypeDef TIM_OCInitStruct;
    TIM_OCStructInit(&TIM_OCInitStruct);
    TIM_OCInitStruct.TIM_OCMode = TIM_OCMode_PWM1; // PWM1模式（高电平有效）
    TIM_OCInitStruct.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStruct.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OC1Init(TIM1, &TIM_OCInitStruct);
    
    // 使能TIM1主输出（高级定时器必须）
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    // 使能TIM1
    TIM_Cmd(TIM1, ENABLE);
    
    // 初始电机停转
    Motor_SetSpeed(0);
}

/**
  * @brief  设置电机速度和方向（接收PID输出）
  * @param  speed: PID输出值（正负代表方向，绝对值代表PWM占空比）
  */
void Motor_SetSpeed(int16_t speed)
{
    if (speed > 0)  // 正转
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_6);
        GPIO_ResetBits(GPIOA, GPIO_Pin_7);
        TIM_SetCompare1(TIM1, speed); // 设置PWM占空比
    }
    else if (speed < 0)  // 反转（取绝对值）
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6);
        GPIO_SetBits(GPIOA, GPIO_Pin_7);
        TIM_SetCompare1(TIM1, -speed);
    }
    else  // 停转
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_6 | GPIO_Pin_7);
        TIM_SetCompare1(TIM1, 0);
    }
}