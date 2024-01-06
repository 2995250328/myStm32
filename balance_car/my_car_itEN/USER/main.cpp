#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScope_DP.h"
#include "GUI.hpp"
#include "Commulink.hpp"
#include "drv_Main.hpp"
#include "show.h"

/*	 
 * WHU-CAR �ӿ�˵��
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
//����ң����صı���
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//���ֹͣ��־λ Ĭ��ֹͣ 
u8 Flag_Stop=1;                 
//���PWM����
int Motor_Left,Motor_Right;  
//�¶ȱ���
int Temperature;        
//���ұ��������������	
int Encoder_Left,Encoder_Right;   
//��ص�ѹ������صı���
float Voltage;                
//ƽ����� ƽ�������� ת��������
float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//��ʱ�͵�����ر���
int delay_50,delay_flag;
//����ģʽ����ͨģʽ��־λ
u8 Flag_Blooth=0,Flag_Normol=1;	
//��ȡʱ��
RCC_ClocksTypeDef RCC_CLK;						
//Z����ٶȼ�  
float Acceleration_Z;         
//PID�������Ŵ�100����
float Balance_Kp=20500,Balance_Kd=110,Velocity_Kp=16000,Velocity_Ki=80,Turn_Kp=4200,Turn_Kd=0;
//�����ٶ�(mm/s)
float Velocity_Left,Velocity_Right;	
//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM����
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
//λ��PID���Ʊ������ٶ�PID���Ʊ���
int Pos_Pwm,Vel_Pwm;
//λ��PIDϵ��
float Position_KP=5,Position_KI=0,Position_KD=0; 	
//�ٶ�PIDϵ��
float Velocity_KP=5,Velocity_KI=0,Velocity_KD=0;
//λ��PID�趨Ŀ����
int Target_Position_L=800,Target_Position_R=800; 			
//�ٶ�PID�趨Ŀ����
int Target_Velocity_L=20,Target_Velocity_R=20;

int main(void)
{	
	//ϵͳʱ�ӳ�ʼ��
	SystemInit();
	//��ʱʱ�ӳ�ʼ��
	delay_init();	
	//��SWD�ӿ�,�رո��ù���
	JTAG_Set(SWD_ENABLE);        
	//����ϵͳ�ж����ȼ�����4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
	//��ʼ���豸����
	init_drv_Main();
	//��ʼ����ʼ����
	sendLedSignal(LEDSignal_Start1);
	//��ȡʱ������
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	

	//������ѭ��
	while(1)
	{	
		LCD_ShowMAIN();
	}
}
