
#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"

#define PI 3.14159265358979f			//PIԲ����
#define Control_Frequency  200.0	//��������ȡƵ��
#define Diameter_67  67.0 				//����ֱ��67mm 
#define EncoderMultiples   4.0 		//��������Ƶ��
#define Encoder_precision  13.0 	//���������� 13��
#define Reduction_Ratio  30.0			//���ٱ�30
#define Perimeter  210.4867 			//�ܳ�����λmm
#define Middle_angle 2						//δ������̬������ƽ���Ϊ2

//��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
extern u8 Way_Angle;                              
//����ң����صı���
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; 
//���ֹͣ��־λ Ĭ��ֹͣ 
extern u8 Flag_Stop;                 
//���PWM����
extern int Motor_Left,Motor_Right;  
//�¶ȱ���
extern int Temperature;             
//��ص�ѹ������صı���
extern int Voltage;                
//ƽ����� ƽ�������� ת��������
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//��ʱ�͵�����ر���
extern u8 delay_50,delay_flag,PID_Send; 						
//Z����ٶȼ�  
extern float Acceleration_Z;         
//PID�������Ŵ�100����
extern float Balance_Kp,Balance_Kd,Velocity_Kp,Velocity_Ki,Turn_Kp,Turn_Kd;
//����ģʽ����ͨģʽ��־λ
extern u8 Flag_Blooth,Flag_Normol;			
//���ұ��������������	
extern int Encoder_Left,Encoder_Right;   
//�����ٶ�(mm/s)
extern float Velocity_Left,Velocity_Right;	
//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM����
extern int Balance_Pwm,Velocity_Pwm,Turn_Pwm;	

/*
 * �������ܣ�ֱ��PD����		
 * ��ڲ�����Angle:�Ƕȣ�Gyro�����ٶ�
 * ����  ֵ��balance��ֱ������PWM
 */	
int Balance(float Angle,float Gyro);
/*
 * �������ܣ��ٶȿ���PWM		
 * ��ڲ�����encoder_left�����ֱ�����������encoder_right�����ֱ���������
 * ����  ֵ���ٶȿ���PWM
 */
int Velocity(int encoder_left,int encoder_right);
/*
 * �������ܣ�ת����� 
 * ��ڲ�����Z��������
 * ����  ֵ��ת�����PWM
 */
int Turn(float gyro);

/*
 * �������ܣ���ֵ��PWM�Ĵ���
 * ��ڲ���������PWM������PWM
 * ����  ֵ����
 */
void Set_Pwm(int motor_left,int motor_right);
/*
 * �������ܣ�����PWM��ֵ 
 * ��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
 * ����  ֵ���޷����ֵ
 */
int PWM_Limit(int IN,int max,int min);
/*
 * �������ܣ������޸�С������״̬ 
 * ��ڲ�������
 * ����  ֵ����
 */
void Key(void);
/*
 * �������ܣ��쳣�رյ��		
 * ��ڲ�����angle��С����ǣ�voltage����ѹ
 * ����  ֵ��1���쳣  0������
 */	
u8 Turn_Off(float angle, int voltage);
	
/*
 * �������ܣ�����ֵ����
 * ��ڲ�����a����Ҫ�������ֵ����
 * ����  ֵ���޷�������
 */	
int myabs(int a);
/*
 * �������ܣ����С���Ƿ�����
 * ��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
 * ����  ֵ��1:С��������  0��С��δ������
 */
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right);
/*
 * �������ܣ����С���Ƿ񱻷���
 * ��ڲ�����ƽ��Ƕȣ���������������ұ���������
 * ����  ֵ��1��С������   0��С��δ����
 */
int Put_Down(float Angle,int encoder_left,int encoder_right);
/*
 * �������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
 * ��ڲ�������
 * ����  ֵ����
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);
/*
 * �������ܣ�ѡ��С������ģʽ
 * ��ڲ�����encoder_left�������������  encoder_right���ұ���������
 * ����  ֵ����
 */
void Choose(int encoder_left,int encoder_right);

void init_drv_ControlSystem();
#endif