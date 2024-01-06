#include "Control.hpp"

#define Debug 0									  //是否进入调试模式
#define Debug_pos 0							  //是否进入位置调试模式
#define Debug_vel 0								//是否进入速度调试模式

#define PI 3.14159265358979f			//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm
#define Middle_angle 2						//未消除静态误差，导致平衡角为2

//获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern u8 Way_Angle;                              
//蓝牙遥控相关的变量
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; 
//电机停止标志位 默认停止 
extern u8 Flag_Stop;                 
//电机PWM变量
extern int Motor_Left,Motor_Right;  
//温度变量
extern int Temperature;             
//电池电压采样相关的变量
extern int Voltage;                
//平衡倾角 平衡陀螺仪 转向陀螺仪
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//延时和调参相关变量
extern int delay_50,delay_flag; 						
//Z轴加速度计  
extern float Acceleration_Z;         
//PID参数（放大100倍）
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
//蓝牙模式、普通模式标志位
extern u8 Flag_Blooth,Flag_Normol;			
//左右编码器的脉冲计数	
extern int Encoder_Left,Encoder_Right;   
//车轮速度(mm/s)
extern float Velocity_Left,Velocity_Right;	
//平衡环PWM变量，速度环PWM变量，转向环PWM变量
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;	
//位置PID
extern float Position_KP,Position_KI,Position_KD; 	
//编码器当前目标数以及设定目标数
extern int Target_Position_L,Target_Position_R; 			
//位置PID控制变量，速度PID控制变量
extern int Pos_Pwm,Vel_Pwm;
//速度PID系数
extern float Velocity_KP,Velocity_KI,Velocity_KD;
//编码器的脉冲计数
extern int Target_Velocity_L,Target_Velocity_R;