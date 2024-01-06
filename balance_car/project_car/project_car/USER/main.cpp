#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Key.hpp"
#include "Commulink.hpp"
#include "drv_MPU6050.hpp"
#include "Control.hpp"	
#include "GUI.hpp"
#include "drv_ADC.hpp"

/*
 *      WHU-CAR
 * 所属单位：武汉大学
 * 版本：1.0
 * 修改时间：2023-03-23
 * 采用LQR平衡算法
 * Brand: WHU
 * Version:1.0
 * Update：2023-03-23
 * All rights reserved ByWX
 */

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

//获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
u8 Way_Angle=2;                              
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
int Voltage;                
//平衡倾角 平衡陀螺仪 转向陀螺仪
float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//延时和调参相关变量
u8 delay_50,delay_flag,PID_Send;
//蓝牙模式、普通模式标志位
u8 Flag_Blooth=0,Flag_Normol=1;							
//Z轴加速度计  
float Acceleration_Z;         
//PID参数（放大100倍）
float Balance_Kp=20500,Balance_Kd=110,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=0;
//车轮速度(mm/s)
float Velocity_Left,Velocity_Right;	
//平衡环PWM变量，速度环PWM变量，转向环PWM变量
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;	

int main(void)
{
	init_Basic();
	//初始化设备驱动
	init_drv_Main();
	//初始化开始声音
	sendLedSignal(LEDSignal_Start1);
	//初始化控制系统
	init_drv_ControlSystem();
	//设置初始化已完成
	setInitializationCompleted();
	//进入主循环
	while(1)
	{
		Commulink_Server();
		//显示
		GUI_Server();		
	}
}
