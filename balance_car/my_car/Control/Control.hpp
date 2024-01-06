#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"


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
