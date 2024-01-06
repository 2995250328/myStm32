#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "GUI.hpp"
#include "Commulink.hpp"
#include "drv_Main.hpp"
#include "show.h"

/*	 
 * WHU-CAR 接口说明
 * PD4  ---> AIN1
 * PC12 ---> AIN2
 * PD11 ---> BIN1
 * PD10 ---> BIN2
 * PE10 ---> STBY
 * PD12 ---> E2A  T4C1
 * PD13 ---> E2B  T4C2
 * PC6  ---> E1A  T3C1
 * PC7  ---> E1B  T3C2
 * PE14 ---> PWMA T1C4
 * PE13 ---> PWMB T1C3
 * PC2  ---> ADC  ADC123IN12 
*/                             
//蓝牙遥控相关的变量
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//电机停止标志位 默认停止 
u8 Flag_Stop=1;                 
//电机PWM变量
int Motor_Left,Motor_Right;  
//温度变量
int Temperature;        
//左右编码器的脉冲计数	
int Encoder_Left,Encoder_Right;   
//电池电压采样相关的变量
float Voltage;                
//平衡倾角 平衡陀螺仪 转向陀螺仪
float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//延时和调参相关变量
int delay_50,delay_flag;
//蓝牙模式、普通模式标志位
u8 Flag_Blooth=0,Flag_Normol=1;	
//获取时钟
RCC_ClocksTypeDef RCC_CLK;						
//Z轴加速度计  
float Acceleration_Z;         
//PID参数（放大100倍）
float Balance_Kp=20500,Balance_Kd=110,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=0;
//车轮速度(mm/s)
float Velocity_Left,Velocity_Right;	
//平衡环PWM变量，速度环PWM变量，转向环PWM变量
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
//位置PID控制变量，速度PID控制变量
int Pos_Pwm,Vel_Pwm;
//位置PID系数
float Position_KP=5,Position_KI=0,Position_KD=0; 	
//速度PID系数
float Velocity_KP=5,Velocity_KI=0,Velocity_KD=0;
//位置PID设定目标数
int Target_Position_L=800,Target_Position_R=800; 			
//速度PID设定目标数
int Target_Velocity_L=20,Target_Velocity_R=20;

int main(void)
{	
	//系统时钟初始化
	SystemInit();
	//延时时钟初始化
	delay_init();	
	//打开SWD接口,关闭复用功能
	JTAG_Set(SWD_ENABLE);        
	//设置系统中断优先级分组4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	//初始化设备驱动
	init_drv_Main();
	//初始化开始声音
	sendLedSignal(LEDSignal_Start1);
	//获取时钟总线
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	

	//进入主循环
	while(1)
	{	
		LCD_ShowMAIN();
	}
}
