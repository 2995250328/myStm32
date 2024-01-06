#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScop_DP.hpp"
#include "GUI.hpp"
#include "Commulink.hpp"
#include "Control.hpp"
#include "drv_Main.hpp"
#include "drv_Timer2.hpp"
#include "drv_LCD.hpp"
#include "Show.hpp"

int Debug=0;								  //�Ƿ�������ģʽ
int Debug_pos=0;							  //�Ƿ����λ�õ���ģʽ
int Debug_vel=0; 							//�Ƿ�����ٶȵ���ģʽ

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
u8 WRONG[]={"wrong!"};
u8 OFF[]={"Solving static bias,don't move"};
u8 VOLT[]={"Please charge"};
//����ң����صı���
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//���ֹͣ��־λ Ĭ��ֹͣ 
u8 Flag_Stop=0; 
//��������flag
int buzz_flag=0;
extern int offset_flag;
extern int Volt_flag;
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
//��ʾflag
int show_flag=0;
//����ģʽ����ͨģʽ��־λ
u8 Flag_Blooth=0,Flag_Normol=1;	
//��ȡʱ��
RCC_ClocksTypeDef RCC_CLK;						
//Z����ٶȼ�  
float Acceleration_Z;         
//PID����
float Balance_Kp=400,Balance_Kd=15.5,Velocity_Kp=77,Velocity_Ki=0.5,Turn_Kp=25,Turn_Kd=1;
//�����ٶ�(mm/s)
float Velocity_Left,Velocity_Right;	
//ƽ�⻷PWM�������ٶȻ�PWM������ת��PWM����
int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
//λ��PID���Ʊ������ٶ�PID���Ʊ���
int Pos_Pwm,Vel_Pwm;
//λ��PIDϵ��
float Position_KP=18.5,Position_KI=0.1,Position_KD=230;  	
//�ٶ�PIDϵ��
float Velocity_KP=30,Velocity_KI=10,Velocity_KD=10;
//λ��PID�趨Ŀ����
int Target_Position_L=0,Target_Position_R=0; 			
//�ٶ�PID�趨Ŀ����
int Target_Velocity_L=7,Target_Velocity_R=5;
float Turn_Target_Tem=0;
float Turn_Target=0;int Turn_360=0;

int main(void)
{	
	//����ϵͳ�ж����ȼ�����4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	//ϵͳʱ�ӳ�ʼ��
	SystemInit();
	//��ʱʱ�ӳ�ʼ��
	delay_init();	
	//��SWD�ӿ�,�رո��ù���
	JTAG_Set(JTAG_SWD_DISABLE);
	JTAG_Set(SWD_ENABLE);        
	//��ʼ���豸����
	init_drv_Main();
	//��ȡʱ������
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	
	//��ʼ����ʱ��2
	Timer2_init();
	//������ѭ��
	while(1)
	{	
		Key();
		//����ģʽ��ѡ��
		if (Debug)
		{
			delay_ms(50);
			DataScope(); 
			delay_ms(50);
			LCD_ShowDebug();
		}
		else
		{
			if(offset_flag==1)//��̬ƫ�ö�ȡ���
			{
				if(show_flag==0)
				{
					LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
					show_flag=1;
				}
				//�����ѹ���� ��ʾ���
				if(Volt_flag==1)
					LCD_ShowString(32,80,VOLT,RED,WHITE,24,0);
				else 
					LCD_ShowMAIN();//��ʾ			
			}
			else if(offset_flag==2)//��̬ƫ�ö�ȡʧ��
			{
				if(buzz_flag==0)
				{
					sendLedSignal(LEDSignal_Start1);
					buzz_flag++;
				}
				Commulink_Server();
				LCD_ShowString(0,128,WRONG,RED,WHITE,32,0);
			}
			else if(offset_flag==0)
			{
				LCD_ShowString(0,80,OFF,RED,WHITE,16,0);
			}
		}
	}
}
