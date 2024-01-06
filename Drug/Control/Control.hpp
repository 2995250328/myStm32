#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"


#define PI 3.14159265358979f			//PI圆周率
#define Control_Frequency  50.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm
#define Middle_angle  -0.5				//机械中值
#define Wheel_Track 15.5          //轮间距15.5cm
#define PWMA TIM1->CCR4
#define PWMB TIM1->CCR3
#define BIN1 PDout(11)
#define BIN2 PDout(10)
#define AIN1 PDout(4)
#define AIN2 PCout(12)
extern int Debug;								  //是否进入调试模式
extern int Debug_pos;							  //是否进入位置调试模式
extern int Debug_vel; 							//是否进入速度调试模式

typedef enum
{
  left_90,
	right_90,
	back_180,
	num_spin1,
	num_spin2
}spin_dir_t;

//获取角度的算法，1：四元数  2：卡尔曼  3：互补滤波
extern u8 Way_Angle;                              
//蓝牙遥控相关的变量
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; 
//电机停止标志位 默认停止 
extern int Stop_Flag;              
//电机PWM变量
extern int Motor_Left,Motor_Right;  
//温度变量
extern int Temperature;             
//电池电压采样相关的变量
extern float Voltage;               
//直行巡线Pwm
extern int Turn_Pwm;
//平衡倾角 平衡陀螺仪 转向陀螺仪
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; 						
//Z轴加速度计  
extern float Acceleration_Z;         
//蓝牙模式、普通模式标志位
extern u8 Flag_Blooth,Flag_Normol;			
//左右编码器的脉冲计数	
extern int Encoder_Left,Encoder_Right;   
//车轮速度(mm/s)
extern float Velocity_Left,Velocity_Right;	
//位置PID
extern float Position_KP_L,Position_KI_L,Position_KD_L; 	
//位置PID
extern float Position_KP_R,Position_KI_R,Position_KD_R; 	
//编码器当前目标数以及设定目标数
extern int Target_Position_L,Target_Position_R; 			
//位置PID控制变量，速度PID控制变量
extern int Pos_Pwm,Vel_Pwm;
//速度PID系数
extern float Velocity_KP_L,Velocity_KI_L,Velocity_KD_L;
//速度PID系数
extern float Velocity_KP_R,Velocity_KI_R,Velocity_KD_R;
//寻迹PID系数
extern float OpenMv_KP,OpenMv_KI,OpenMv_KD;
//编码器的脉冲计数 增量pid的目标值
extern int Target_Velocity;
extern int Position_L,Position_R;//编码器总计数
//编码器计数转换为实际距离
extern int Location_CM_L,Location_CM_R;
//速度位置pid控制变量
extern int LV_PWM_L,LV_PWM_R;
//直行时的Flag
extern int Straight_Flag;
//停止的Flag
extern int Stop_Flag;
//转向的FLag
extern int Spin_Flag;

/*
 * 函数功能：设置前进固定距离	
 * 入口参数：目标距离
 * 返回  值：无
 */	
void Car_Go(int Target_Location);
/*
 * 函数功能：设置转向角度	
 * 入口参数：目标角度 枚举型
 * 返回  值：无
 */	
void Car_Spin(spin_dir_t Turn);
/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID_L (int Position,int Target);
/*
 * 函数功能：增量（速度）PID控制	
 * 入口参数：编码器测量值，目标速度
 * 返回  值：电机控制的PWM
 */
int Speed_PID_L(int Encoder,int Target);
/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID_R (int Position,int Target);
/*
 * 函数功能：增量（速度）PID控制	
 * 入口参数：编码器测量值，目标速度
 * 返回  值：电机控制的PWM
 */
int Speed_PID_R(int Encoder,int Target);
/*
 * 函数功能：位置速度串级pid
 * 入口参数：编码器测量值，目标位置
 * 返回  值：电机控制的PWM
 */
int Location_Speed_L(int Encoder,int Location,int Target_P,int Target_V);
/*
 * 函数功能：位置速度串级pid
 * 入口参数：编码器测量值，目标位置
 * 返回  值：电机控制的PWM
 */
int Location_Speed_R(int Encoder,int Location,int Target_P,int Target_V);
/*
 * 函数功能：赋值给PWM寄存器
 * 入口参数：左轮PWM、右轮PWM
 * 返回  值：无
 */
void Set_Pwm(int motor_left,int motor_right);
/*
 * 函数功能：限制PWM赋值 
 * 入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
 * 返回  值：限幅后的值
 */
int PWM_Limit(int IN,int max,int min);
/*
 * 函数功能：绝对值函数
 * 入口参数：a：需要计算绝对值的数
 * 返回  值：无符号整型
 */	
int myabs(int a);
/*
 * 函数功能：编码器读数转换为速度（mm/s）
 * 入口参数：无
 * 返回  值：无
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);

float OPENMV_Turn_PID(int x_position);

#endif
