#include "stm32f10x.h"                  // Device header
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include "Encoder.h"
#include "Encoder2.h"
#include "Serial.h"
#include "OLED.h"
#include "PID.h"
#include "Motor.h"
#include "Motor2.h"
#include "AnglePID.h"
#include "Key.h"

extern PID_HandleTypeDef pid;
extern AnglePID_HandleTypeDef anglePid;
extern int32_t Motor1_Angle;
extern int32_t Motor2_Angle;
//串口指令缓冲区（储存@speed%格式指令）
char CmdBuf[20];
uint8_t CmdLen = 0;
extern uint8_t RxData;

uint8_t Serial_RxData;		//定义串口接收的数据变量
uint8_t Serial_RxFlag;		//定义串口接收的标志位变量

/**
  * 函    数：串口初始化
  * 参    数：无
  * 返 回 值：无
  */
void Serial_Init(void)
{
	/*开启时钟*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//开启USART1的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//开启GPIOA的时钟
	
	/*GPIO初始化*/
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA9引脚初始化为复用推挽输出
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//将PA10引脚初始化为上拉输入
	
	/*USART初始化*/
	USART_InitTypeDef USART_InitStructure;					//定义结构体变量
	USART_InitStructure.USART_BaudRate = 9600;				//波特率
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;	//硬件流控制，不需要
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;	//模式，发送模式和接收模式均选择
	USART_InitStructure.USART_Parity = USART_Parity_No;		//奇偶校验，不需要
	USART_InitStructure.USART_StopBits = USART_StopBits_1;	//停止位，选择1位
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;		//字长，选择8位
	USART_Init(USART1, &USART_InitStructure);				//将结构体变量交给USART_Init，配置USART1
	
	/*中断输出配置*/
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);			//开启串口接收数据的中断
	
	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);			//配置NVIC为分组2
	
	/*NVIC配置*/
	NVIC_InitTypeDef NVIC_InitStructure;					//定义结构体变量
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;		//选择配置NVIC的USART1线
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//指定NVIC线路使能
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;		//指定NVIC线路的抢占优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//指定NVIC线路的响应优先级为1
	NVIC_Init(&NVIC_InitStructure);							//将结构体变量交给NVIC_Init，配置NVIC外设
	
	/*USART使能*/
	USART_Cmd(USART1, ENABLE);								//使能USART1，串口开始运行
}

/**
  * TIM4初始化（10ms定时中断，替换原TIM3避免冲突）
  */
void Timer4_Init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//开启TIM4时钟（APB1总线）
	
	/*10ms定时配置（36MHz时钟→1kHz计数）*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Prescaler = 35999;			//36MHz/(35999+1)=1kHz
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 99;					//1kHz×100次=10ms
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
	
	/*NVIC配置（优先级低于串口）*/
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;			//TIM4中断通道
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);	//开启更新中断
	TIM_Cmd(TIM4, ENABLE);						//使能TIM4
}

/**
  * TIM4中断服务函数（10ms读编码器+串口发送）
  */
/** 
	void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        // 1. 读取编码器反馈值
        int16_t feedback = Encoder_Get();
        // 2. 更新PID反馈并计算输出
        PID_UpdateFeedback(&pid, feedback);
        int16_t pidOut = PID_Calculate(&pid);
        // 3. 控制电机
        Motor_SetSpeed(pidOut);
        
        // 4. OLED显示
        OLED_ShowSignedNum(3, 10, feedback, 4);  // 反馈速度
        OLED_ShowSignedNum(4, 8, pidOut, 4);     // PID输出
        
        // 5. 串口发送（格式：Target:XXX,Feedback:XXX,Output:XXX\r\n，上位机可同时绘图）
        Serial_Printf("Target:%d,Feedback:%d,Output:%d\r\n", 
                      pid.Target, feedback, pidOut);
        
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    }
}
*/
       // 显示当前模式
// TIM4中断服务函数（完整版本）
void TIM4_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
    {
        // 1. 读取双编码器数据（电机1=原编码器，电机2=新增编码器2）
        int16_t M1_Feedback = Encoder_Get();  // 电机1：原有编码器（PA6/PA7-TIM3）
        int16_t M2_Feedback = Encoder2_Get(); // 电机2：新增编码器2（PB6/PB7-TIM4，避免冲突）
        
        // 2. 累计双电机角度（编码器总增量 = 实际角度）
        Motor1_Angle += M1_Feedback;
        Motor2_Angle += M2_Feedback;
        
        // 3. 模式判断：0=速度控制（原功能），1=角度跟随（新功能）
        uint8_t mode = Key_GetMode();
        if (mode == 0)
        {
            // 模式0：电机1速度PID控制，电机2停转
            PID_UpdateFeedback(&pid, M1_Feedback);
            int16_t pidOut = PID_Calculate(&pid);
            Motor_SetSpeed(pidOut);
            Motor2_SetSpeed(0);
            
            // OLED显示（速度模式）
            OLED_ShowString(2, 1, "Target(Speed):");
            OLED_ShowSignedNum(2, 14, pid.Target, 4);
            OLED_ShowString(3, 1, "M1_Speed:");
            OLED_ShowSignedNum(3, 11, M1_Feedback, 4);
            OLED_ShowString(4, 1, "M2_Speed:");
            OLED_ShowSignedNum(4, 11, 0, 4);
            
            // 串口发送速度数据（上位机可绘图）
            Serial_Printf("Mode:Speed,Target:%d,M1_Speed:%d,M2_Speed:0\r\n", 
                          pid.Target, M1_Feedback);
        }
        else
        {
            // 模式1：电机1角度→电机2跟随（核心功能）
            AnglePID_SetTarget(&anglePid, Motor1_Angle);  // 电机1角度为目标
            AnglePID_UpdateFeedback(&anglePid, Motor2_Angle); // 电机2角度为反馈
            int16_t angleOut = AnglePID_Calculate(&anglePid);
            Motor2_SetSpeed(angleOut);  // 电机2执行跟随控制
            Motor_SetSpeed(0);          // 电机1自由转动（手动操作）
            
            // OLED显示（角度模式）
            OLED_ShowString(2, 1, "Target(Angle):");
            OLED_ShowSignedNum(2, 14, Motor1_Angle, 6);
            OLED_ShowString(3, 1, "M1_Angle:");
            OLED_ShowSignedNum(3, 11, Motor1_Angle, 6);
            OLED_ShowString(4, 1, "M2_Angle:");
            OLED_ShowSignedNum(4, 11, Motor2_Angle, 6);
            OLED_ShowString(4, 18, "Diff:");
            OLED_ShowSignedNum(4, 23, Motor1_Angle - Motor2_Angle, 3); // 角度差（精度检测）
            
            // 串口发送角度数据（上位机对比双电机角度）
            Serial_Printf("Mode:Angle,M1_Angle:%d,M2_Angle:%d,Diff:%d\r\n", 
                          Motor1_Angle, Motor2_Angle, Motor1_Angle - Motor2_Angle);
        }
        
        TIM_ClearITPendingBit(TIM4, TIM_IT_Update); // 清除中断标志位
    }
}


/**
  * 函    数：串口发送一个字节
  * 参    数：Byte 要发送的一个字节
  * 返 回 值：无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);		//将字节数据写入数据寄存器，写入后USART自动生成时序波形
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);	//等待发送完成
	/*下次写入数据寄存器会自动清除发送完成标志位，故此循环后，无需清除标志位*/
}

/**
  * 函    数：串口发送一个数组
  * 参    数：Array 要发送数组的首地址
  * 参    数：Length 要发送数组的长度
  * 返 回 值：无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++)		//遍历数组
	{
		Serial_SendByte(Array[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：串口发送一个字符串
  * 参    数：String 要发送字符串的首地址
  * 返 回 值：无
  */
void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)//遍历字符数组（字符串），遇到字符串结束标志位后停止
	{
		Serial_SendByte(String[i]);		//依次调用Serial_SendByte发送每个字节数据
	}
}

/**
  * 函    数：次方函数（内部使用）
  * 返 回 值：返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;	//设置结果初值为1
	while (Y --)			//执行Y次
	{
		Result *= X;		//将X累乘到结果
	}
	return Result;
}

/**
  * 函    数：串口发送数字
  * 参    数：Number 要发送的数字，范围：0~4294967295
  * 参    数：Length 要发送数字的长度，范围：0~10
  * 返 回 值：无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++)		//根据数字长度遍历数字的每一位
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');	//依次调用Serial_SendByte发送每位数字
	}
}

/**
  * 函    数：使用printf需要重定向的底层函数
  * 参    数：保持原始格式即可，无需变动
  * 返 回 值：保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);			//将printf的底层重定向到自己的发送字节函数
	return ch;
}

/**
  * 函    数：自己封装的prinf函数
  * 参    数：format 格式化字符串
  * 参    数：... 可变的参数列表
  * 返 回 值：无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];				//定义字符数组
	va_list arg;					//定义可变参数列表数据类型的变量arg
	va_start(arg, format);			//从format开始，接收参数列表到arg变量
	vsprintf(String, format, arg);	//使用vsprintf打印格式化字符串和参数列表到字符数组中
	va_end(arg);					//结束变量arg
	Serial_SendString(String);		//串口发送字符数组（字符串）
}

/**
  * 函    数：获取串口接收标志位
  * 参    数：无
  * 返 回 值：串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)			//如果标志位为1
	{
		Serial_RxFlag = 0;
		return 1;					//则返回1，并自动清零标志位
	}
	return 0;						//如果标志位为0，则返回0
}

/**
  * 函    数：获取串口接收的数据
  * 参    数：无
  * 返 回 值：接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxData;			//返回接收的数据变量
}

/**
  * 函    数：USART1中断函数
  * 参    数：无
  * 返 回 值：无
  * 注意事项：此函数为中断函数，无需调用，中断触发后自动执行
  *           函数名为预留的指定名称，可以从启动文件复制
  *           请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART1_IRQHandler(void)
{
	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)		//判断是否是USART1的接收事件触发的中断
	{
		uint8_t ch = USART_ReceiveData(USART1);
		Serial_RxData = ch;										//读取数据寄存器，存放在接收的数据变量
		Serial_RxFlag = 1;										//置接收标志位变量为1
		
		if (ch == '@')
		{
			CmdLen = 0;
			memset(CmdBuf, 0, sizeof(CmdBuf));
		}
		else if(ch == '%')
		{
			CmdBuf[CmdLen] = '\0';
			//匹配speed指令
			if (strstr(CmdBuf,"speed") != NULL)
			{
				int16_t target = atoi(CmdBuf + 5);		//提取速度值，跳过speed
				PID_SetTarget(&pid, target);
				OLED_ShowSignedNum(2,8,target,4);
				Serial_Printf("Set Target Speed: %d\r\n", target);
			}
		}
		else if (CmdLen < 19)
		{
			CmdBuf[CmdLen++] = ch;
		}
		USART_ClearITPendingBit(USART1, USART_IT_RXNE);			//清除USART1的RXNE标志位
																//读取数据寄存器会自动清除此标志位
																//如果已经读取了数据寄存器，也可以不执行此代码
	}
}
