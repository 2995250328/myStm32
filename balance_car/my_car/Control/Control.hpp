#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"


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

void init_drv_ControlSystem();
#endif
