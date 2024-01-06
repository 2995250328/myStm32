#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define Debug 1									  //是否进入调试模式
#define Debug_pos 0							  //是否进入位置调试模式
#define Debug_vel 1 							//是否进入速度调试模式

#define PI 3.14159265358979f			//PI圆周率
#define Control_Frequency  200.0	//编码器读取频率
#define Diameter_67  67.0 				//轮子直径67mm 
#define EncoderMultiples   4.0 		//编码器倍频数
#define Encoder_precision  13.0 	//编码器精度 13线
#define Reduction_Ratio  30.0			//减速比30
#define Perimeter  210.4867 			//周长，单位mm
#define Middle_angle 0						//未消除静态误差，导致平衡角为2
#define PWMA TIM1->CCR4
#define PWMB TIM1->CCR3
#define BIN1 PDout(11)
#define BIN2 PDout(10)
#define AIN1 PDout(4)
#define AIN2 PCout(12)

typedef enum
{
  left_90,
	right_90,
	back_180
}spin_dir_t;

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
extern float Voltage;                
//平衡倾角 平衡陀螺仪 转向陀螺仪
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; 						
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

/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID (int Position,int Target);
/*
 * 函数功能：增量（速度）PID控制	
 * 入口参数：编码器测量值，目标速度
 * 返回  值：电机控制的PWM
 */
int Incremental_PI (int Encoder,int Target);
/*
 * 函数功能：直立PD控制		
 * 入口参数：Angle:角度；Gyro：角速度
 * 返回  值：balance：直立控制PWM
 */	
int Balance(float Angle,float Gyro);
/*
 * 函数功能：速度控制PWM		
 * 入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
 * 返回  值：速度控制PWM
 */
int Velocity(int encoder_left,int encoder_right);
/*
 * 函数功能：转向控制 
 * 入口参数：Z轴陀螺仪
 * 返回  值：转向控制PWM
 */
int Turn(float gyro);

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
 * 函数功能：按键修改小车运行状态 
 * 入口参数：无
 * 返回  值：无
 */
void Key(void);
/*
 * 函数功能：异常关闭电机		
 * 入口参数：angle：小车倾角；voltage：电压
 * 返回  值：1：异常  0：正常
 */	
u8 Turn_Off(float angle, int voltage);
	
/*
 * 函数功能：绝对值函数
 * 入口参数：a：需要计算绝对值的数
 * 返回  值：无符号整型
 */	
int myabs(int a);
/*
 * 函数功能：检测小车是否被拿起
 * 入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
 * 返回  值：1:小车被拿起  0：小车未被拿起
 */
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
/*
 * 函数功能：检测小车是否被放下
 * 入口参数：平衡角度；左编码器读数；右编码器读数
 * 返回  值：1：小车放下   0：小车未放下
 */
int Put_Down(float Angle,int encoder_left,int encoder_right);
/*
 * 函数功能：编码器读数转换为速度（mm/s）
 * 入口参数：无
 * 返回  值：无
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
/*
 * 函数功能：选择小车运行模式
 * 入口参数：encoder_left：左编码器读数  encoder_right：右编码器读数
 * 返回  值：无
 */
void Choose(int encoder_left,int encoder_right);


#endif
