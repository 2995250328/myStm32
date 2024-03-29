#include <stdio.h>
#include "drv_Main.hpp"
#include "filter.h"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Key.hpp"
#include "drv_MPU6050.hpp"
#include "drv_Encoder.hpp"
#include "drv_Encoder_exti.hpp"
#include "drv_ExtIIC.hpp"
#include "drv_PwmOut.hpp"
#include "drv_ADC.hpp"
#include "drv_Sensors.hpp"
#include "drv_Timer2.hpp"
#include "drv_OpenMv.hpp"
#include "drv_K210.hpp"
#include "drv_Blooth.hpp"
#include "drv_Check.hpp"
#include "Commulink.hpp"
#include "inv_mpu.h"
#include "usart.h"
#include "GUI.hpp"
void init_drv_Main()
{
	//初始化串口
	init_drv_Uart1(115200);
	init_drv_Uart3(115200);
	init_drv_Uart4(115200);
	//LED初始化函数
	init_drv_LED();
	//检查装药的初始化
	init_drv_Check();
	//初始化屏幕
	init_drv_LCD();
	//初始化按键
	EXTIX_Init();
	//初始化屏幕为白色
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	//初始化电机接口
	init_drv_Motor();
	//初始化PWM
	init_drv_PWMOut();
	//初始化传感器
//	init_drv_Sensors();
	//初始化编码器
	drv_Encoder_Init_TIM3();
	drv_Encoder_Init_TIM4();
	//初始化ADC
	init_drv_ADC();
}