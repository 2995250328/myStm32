#include "Control.hpp"

#define Debug 0									  //�Ƿ�������ģʽ
#define Debug_pos 0							  //�Ƿ����λ�õ���ģʽ
#define Debug_vel 0								//�Ƿ�����ٶȵ���ģʽ

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
extern int delay_50,delay_flag; 						
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
//λ��PID
extern float Position_KP,Position_KI,Position_KD; 	
//��������ǰĿ�����Լ��趨Ŀ����
extern int Target_Position_L,Target_Position_R; 			
//λ��PID���Ʊ������ٶ�PID���Ʊ���
extern int Pos_Pwm,Vel_Pwm;
//�ٶ�PIDϵ��
extern float Velocity_KP,Velocity_KI,Velocity_KD;
//���������������
extern int Target_Velocity_L,Target_Velocity_R;