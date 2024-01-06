#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"


#define PI 3.14159265358979f			//PIԲ����
#define Control_Frequency  50.0	//��������ȡƵ��
#define Diameter_67  67.0 				//����ֱ��67mm 
#define EncoderMultiples   4.0 		//��������Ƶ��
#define Encoder_precision  13.0 	//���������� 13��
#define Reduction_Ratio  30.0			//���ٱ�30
#define Perimeter  210.4867 			//�ܳ�����λmm
#define Middle_angle  -0.5				//��е��ֵ
#define Wheel_Track 15.5          //�ּ��15.5cm
#define PWMA TIM1->CCR4
#define PWMB TIM1->CCR3
#define BIN1 PDout(11)
#define BIN2 PDout(10)
#define AIN1 PDout(4)
#define AIN2 PCout(12)
extern int Debug;								  //�Ƿ�������ģʽ
extern int Debug_pos;							  //�Ƿ����λ�õ���ģʽ
extern int Debug_vel; 							//�Ƿ�����ٶȵ���ģʽ

typedef enum
{
  left_90,
	right_90,
	back_180,
	num_spin1,
	num_spin2
}spin_dir_t;

//��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
extern u8 Way_Angle;                              
//����ң����صı���
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity; 
//���ֹͣ��־λ Ĭ��ֹͣ 
extern int Stop_Flag;              
//���PWM����
extern int Motor_Left,Motor_Right;  
//�¶ȱ���
extern int Temperature;             
//��ص�ѹ������صı���
extern float Voltage;               
//ֱ��Ѳ��Pwm
extern int Turn_Pwm;
//ƽ����� ƽ�������� ת��������
extern float Angle_Balance,Gyro_Balance,Gyro_Turn; 						
//Z����ٶȼ�  
extern float Acceleration_Z;         
//����ģʽ����ͨģʽ��־λ
extern u8 Flag_Blooth,Flag_Normol;			
//���ұ��������������	
extern int Encoder_Left,Encoder_Right;   
//�����ٶ�(mm/s)
extern float Velocity_Left,Velocity_Right;	
//λ��PID
extern float Position_KP_L,Position_KI_L,Position_KD_L; 	
//λ��PID
extern float Position_KP_R,Position_KI_R,Position_KD_R; 	
//��������ǰĿ�����Լ��趨Ŀ����
extern int Target_Position_L,Target_Position_R; 			
//λ��PID���Ʊ������ٶ�PID���Ʊ���
extern int Pos_Pwm,Vel_Pwm;
//�ٶ�PIDϵ��
extern float Velocity_KP_L,Velocity_KI_L,Velocity_KD_L;
//�ٶ�PIDϵ��
extern float Velocity_KP_R,Velocity_KI_R,Velocity_KD_R;
//Ѱ��PIDϵ��
extern float OpenMv_KP,OpenMv_KI,OpenMv_KD;
//��������������� ����pid��Ŀ��ֵ
extern int Target_Velocity;
extern int Position_L,Position_R;//�������ܼ���
//����������ת��Ϊʵ�ʾ���
extern int Location_CM_L,Location_CM_R;
//�ٶ�λ��pid���Ʊ���
extern int LV_PWM_L,LV_PWM_R;
//ֱ��ʱ��Flag
extern int Straight_Flag;
//ֹͣ��Flag
extern int Stop_Flag;
//ת���FLag
extern int Spin_Flag;

/*
 * �������ܣ�����ǰ���̶�����	
 * ��ڲ�����Ŀ�����
 * ����  ֵ����
 */	
void Car_Go(int Target_Location);
/*
 * �������ܣ�����ת��Ƕ�	
 * ��ڲ�����Ŀ��Ƕ� ö����
 * ����  ֵ����
 */	
void Car_Spin(spin_dir_t Turn);
/*
 * �������ܣ�λ��PID����	
 * ��ڲ�����Positon ��ǰλ�ü����������� Target Ŀ��λ��
 * ����  ֵ��������Ƶ�PWM
 */	
int Position_PID_L (int Position,int Target);
/*
 * �������ܣ��������ٶȣ�PID����	
 * ��ڲ���������������ֵ��Ŀ���ٶ�
 * ����  ֵ��������Ƶ�PWM
 */
int Speed_PID_L(int Encoder,int Target);
/*
 * �������ܣ�λ��PID����	
 * ��ڲ�����Positon ��ǰλ�ü����������� Target Ŀ��λ��
 * ����  ֵ��������Ƶ�PWM
 */	
int Position_PID_R (int Position,int Target);
/*
 * �������ܣ��������ٶȣ�PID����	
 * ��ڲ���������������ֵ��Ŀ���ٶ�
 * ����  ֵ��������Ƶ�PWM
 */
int Speed_PID_R(int Encoder,int Target);
/*
 * �������ܣ�λ���ٶȴ���pid
 * ��ڲ���������������ֵ��Ŀ��λ��
 * ����  ֵ��������Ƶ�PWM
 */
int Location_Speed_L(int Encoder,int Location,int Target_P,int Target_V);
/*
 * �������ܣ�λ���ٶȴ���pid
 * ��ڲ���������������ֵ��Ŀ��λ��
 * ����  ֵ��������Ƶ�PWM
 */
int Location_Speed_R(int Encoder,int Location,int Target_P,int Target_V);
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
 * �������ܣ�����ֵ����
 * ��ڲ�����a����Ҫ�������ֵ����
 * ����  ֵ���޷�������
 */	
int myabs(int a);
/*
 * �������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
 * ��ڲ�������
 * ����  ֵ����
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right);

float OPENMV_Turn_PID(int x_position);

#endif
