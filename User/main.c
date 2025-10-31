#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Encoder.h"
#include "Encoder2.h"
#include "PID.h"
#include "Motor.h"
#include "Motor2.h"
#include "Key.h"
#include "AnglePID.h"

PID_HandleTypeDef pid;
AnglePID_HandleTypeDef anglePid;
int32_t Motor1_Angle = 0;
int32_t Motor2_Angle = 0;
uint8_t RxData = 0;			//定义用于接收串口数据的变量

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	
//	/*显示静态字符串*/
//	OLED_ShowString(1, 1, "RxData:");

	/*串口初始化*/
	Serial_Init();		//串口初始化
	
	Encoder_Init();		//编码器初始化
	Encoder2_Init();
	Key_Init();
	Motor_Init();		//电机初始化
	Motor2_Init();
	Timer4_Init();		//10ms定时中断初始化
	PID_Init(&pid);		//PID初始化
	AnglePID_Init(&anglePid);
	
//	OLED_ShowString(1,1,"RxData:");
//	OLED_ShowString(2,1,"EncoderCNT:");
//	OLED_ShowString(3,1,"Feedback:");
//	OLED_ShowString(4,1,"Output:");
	OLED_ShowString(1, 1, "Mode:");
	OLED_ShowString(1,6, "Speed Ctrl");
    OLED_ShowString(2, 1, "Target:");
    OLED_ShowString(3, 1, "M1_Angle:");
    OLED_ShowString(4, 1, "M2_Angle:");
	
	
	
	while (1)
	{
		//按键扫描
		Key_GetNum();
		
	 // 显示当前模式
 //       uint8_t mode = Key_GetMode();
 /*       if (mode == 0)
        {
            OLED_ShowString(1, 6, "Speed Ctrl");  // 模式0：速度控制（原功能）
        }
        else
        {
            OLED_ShowString(1, 6, "Angle Follow");// 模式1：角度跟随（新功能）
      // 显示两电机角度差（精度检测）
            OLED_ShowSignedNum(4, 12, Motor1_Angle - Motor2_Angle, 3);
        }
		*/
		if (Serial_GetRxFlag() == 1)			//检查串口接收数据的标志位
		{
			RxData = Serial_GetRxData();		//获取串口接收的数据
			Serial_SendByte(RxData);			//串口将收到的数据回传回去，用于测试
//			OLED_ShowHexNum(1, 8, RxData, 2);	//显示串口接收的数据
			OLED_ShowHexNum(2,14,RxData,2);
		}
	}
}
