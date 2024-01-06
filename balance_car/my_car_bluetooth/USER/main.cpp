#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScop_DP.hpp"
#include "GUI.hpp"
#include "Commulink.hpp"
#include "Control.hpp"
#include "drv_Main.hpp"
#include "drv_Timer2.hpp"
#include "drv_LCD.hpp"
#include "Show.hpp"

int Debug=0;								  //是否进入调试模式
int Debug_pos=0;							  //是否进入位置调试模式
int Debug_vel=0; 							//是否进入速度调试模式

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
u8 WRONG[]={"wrong!"};
u8 OFF[]={"Solving static bias,don't move"};
u8 VOLT[]={"Please charge"};
//蓝牙遥控相关的变量
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//电机停止标志位 默认停止 
u8 Flag_Stop=0; 
//声音报警flag
int buzz_flag=0;
extern int offset_flag;
extern int Volt_flag;
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
//显示flag
int show_flag=0;
//蓝牙模式、普通模式标志位
u8 Flag_Blooth=0,Flag_Normol=1;	
//获取时钟
RCC_ClocksTypeDef RCC_CLK;						
//Z轴加速度计  
float Acceleration_Z;         
//PID参数
float Balance_Kp=400,Balance_Kd=15.5,Velocity_Kp=77,Velocity_Ki=0.5,Turn_Kp=25,Turn_Kd=1;
//车轮速度(mm/s)
float Velocity_Left,Velocity_Right;	
//平衡环PWM变量，速度环PWM变量，转向环PWM变量
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
//位置PID控制变量，速度PID控制变量
int Pos_Pwm,Vel_Pwm;
//位置PID系数
float Position_KP=18.5,Position_KI=0.1,Position_KD=230;  	
//速度PID系数
float Velocity_KP=30,Velocity_KI=10,Velocity_KD=10;
//位置PID设定目标数
int Target_Position_L=0,Target_Position_R=0; 			
//速度PID设定目标数
int Target_Velocity_L=7,Target_Velocity_R=5;
float Turn_Target_Tem=0;
float Turn_Target=0;int Turn_360=0;

int main(void)
{	
	//设置系统中断优先级分组4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	//系统时钟初始化
	SystemInit();
	//延时时钟初始化
	delay_init();	
	//打开SWD接口,关闭复用功能
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);        
	//初始化设备驱动
	init_drv_Main();
	//获取时钟总线
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	
	//初始化定时器2
	Timer2_init();
	//进入主循环
	while(1)
	{	
		Key();
		//调试模式的选择
		if (Debug)
		{
			delay_ms(50);
			DataScope(); 
			delay_ms(50);
			LCD_ShowDebug();
		}
		else
		{
			if(offset_flag==1)//静态偏置读取完成
			{
				if(show_flag==0)
				{
					LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
					show_flag=1;
				}
				//如果电压过低 提示充电
				if(Volt_flag==1)
					LCD_ShowString(32,80,VOLT,RED,WHITE,24,0);
				else 
					LCD_ShowMAIN();//显示			
			}
			else if(offset_flag==2)//静态偏置读取失败
			{
				if(buzz_flag==0)
				{
					sendLedSignal(LEDSignal_Start1);
					buzz_flag++;
				}
				Commulink_Server();
				LCD_ShowString(0,128,WRONG,RED,WHITE,32,0);
			}
			else if(offset_flag==0)
			{
				LCD_ShowString(0,80,OFF,RED,WHITE,16,0);
			}
		}
	}
}
