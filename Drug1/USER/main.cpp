#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScop_DP.hpp"
#include "GUI.hpp"
#include "Control.hpp"
#include "drv_Main.hpp"
#include "drv_Timer2.hpp"
#include "drv_Key.hpp"
#include "drv_OpenMv.hpp"
#include "drv_K210.hpp"
#include "drv_Blooth.hpp"
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
 PB9 �̵�
 PB8�Ƶ�
*/                 
//����ң����صı���
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//���ֹͣ��־λ Ĭ��ֹͣ 
u8 Flag_Stop=0;     
//���PWM����
int Motor_Left,Motor_Right;  
//�¶ȱ���
int Temperature;        
//���ұ��������������	
int Encoder_Left,Encoder_Right;  
//���ұ���������ͳ��
int Position_L=0,Position_R=0;
//��������������Ϊʵ�ʾ���
int Location_CM_L=0,Location_CM_R=0;
//ֱ��Ѳ��Pwm
int Turn_Pwm=0;
//��ص�ѹ������صı���
float Voltage;                
//��ʱ�͵�����ر���
int delay_50,delay_flag;
//��ȡʱ��
RCC_ClocksTypeDef RCC_CLK;						
//Z����ٶȼ�  
float Acceleration_Z;   
//ƽ����� ƽ�������� ת��������
float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//�����ٶ�(mm/s)
float Velocity_Left,Velocity_Right;	
//λ��PID���Ʊ������ٶ�PID���Ʊ���
int Pos_Pwm,Vel_Pwm;
//�ٶ�λ��pid���Ʊ���
int LV_PWM_L=0,LV_PWM_R=0;
//λ��PIDϵ��
float Position_KP_L=74.79,Position_KI_L=0.52,Position_KD_L=185; 	
//�ٶ�PIDϵ��
float Velocity_KP_L=75,Velocity_KI_L=9.5,Velocity_KD_L=35.6; 
//λ��PIDϵ��
float Position_KP_R=99.5,Position_KI_R=0.62,Position_KD_R=185; 
//�ٶ�PIDϵ�� 
float Velocity_KP_R=125,Velocity_KI_R=9.5,Velocity_KD_R=35.6; 
//Ѱ��PIDϵ��	
float OpenMv_KP=20,OpenMv_KI=0.43,OpenMv_KD=30;
//λ��PID�趨Ŀ����
int Target_Position_L=0,Target_Position_R=0; 			
//�ٶ�PID�趨Ŀ����
int Target_Velocity=35;
//��k210����������ʶ����ر���
int LorR=0;//�ж�Ŀ������λ��������ҵı��� 1Ϊ�� 2Ϊ��
int Num=0;//ʶ�𵽵�����
//���͸�k210������
u8 K210_Uart_Tx_Buffer[8] = {0};
u8 K210_Uart_Rx_Buffer[8]={0};
u8 K210_Uart_Rx_Index=0;
u8 K210_Rx_Data[8];
//�����������
u8 Blooth_Uart_Tx_Buffer[8] = {0};
u8 Blooth_Uart_Rx_Buffer[8]={0};
u8 Blooth_Uart_Rx_Index=0;
u8 Blooth_Rx_Data[8];
//OpenMV�������
u8 OpenMv_Uart_Tx_Buffer[8] = {0};
u8 OpenMv_Uart_Rx_Buffer[8]={0};
u8 OpenMv_Uart_Rx_Index=0;
u8 OpenMv_Rx_Data[8];
//����ͣ��
int Finish=1;
//�������ݵ���ʱ
int Delay_Transmit=0;
//Ŀ�귿��
int Target_Room=0;
//�����ͷ��ص�״̬���� ����״̬��
int State_Go=0;
int State_Back=0;
//ֱ��ʱ��Flag
int Straight_Flag=0;
//ֹͣ��Flag
int Stop_Flag=0;
//ת���FLag
int Spin_Flag=0;
//װҩ��Flag
int Load_Flag=0;
//Զ��С�Ƕ�ת��ʶ������
int Far_Flag=0;
//װҩ����ʱ��flag
int Delay_Flag=0;
//����flag
int Back_Flag=0;
//��ʼʱ������Ϣ��K210��Flag
int Init_Flag=0;
int Little_Flag=0;
extern int delay_count;
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
	//k210�������ݰ�
	K210_Uart_Tx_Buffer[0]=0x2c;
	K210_Uart_Tx_Buffer[1]=0x12;
	K210_Uart_Tx_Buffer[2]=0x01;
	K210_Uart_Tx_Buffer[7]=0x5b;
	//�����������ݰ�
	Blooth_Uart_Tx_Buffer[0]=0x2c;
	Blooth_Uart_Tx_Buffer[1]=0x12;
	Blooth_Uart_Tx_Buffer[2]=0;
	Blooth_Uart_Tx_Buffer[4]=0;
	Blooth_Uart_Tx_Buffer[7]=0x5b;
	//��ʼ����ʱ��2
	Timer2_init();
	//������ѭ��
	while(1)
	{	
		//����ģʽ��ѡ��
		if (Debug)
		{
			delay_ms(50);
			DataScope(); 
			delay_ms(50);
		}
		else
		{		
			LCD_ShowMAIN();		
			Blooth_Uart_Tx_Buffer[3]=Target_Room;//һֱ����Ŀ�귿��
			if(Init_Flag<5)
			{
				K210_Transmit(K210_Uart_Tx_Buffer);
				Init_Flag++;
			}
			else if(Init_Flag==5)
			{
				Blooth_Transmit(Blooth_Uart_Tx_Buffer);
				K210_Uart_Tx_Buffer[2]=0;
				K210_Transmit(K210_Uart_Tx_Buffer);
			}

			if(Delay_Flag==1)
			{
				delay_ms(500);
				Delay_Flag=0;
			}
			if(Load_Flag==1)
			{
				PBout(9)=0;
				if(Target_Room=='A')
				{
					switch(State_Go)
					{
						case 0:
							Car_Go(89);
							State_Go++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								Car_Spin(left_90);
								State_Go++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Car_Go(33); 
								State_Go++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								Finish=0;
								Straight_Flag=0;
								Spin_Flag=0;
								PEout(0)=0;
							}
						break;
					}
				}			
				else if(Target_Room=='B')
				{
					switch(State_Go)
					{
						case 0:
							Car_Go(89);
							State_Go++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Car_Go(33);
								State_Go++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								Finish=0;
								PEout(0)=0;
								Straight_Flag=0;
								Spin_Flag=0;
							}
						break;
					}
				}			
				else 
				{
					switch(State_Go)
					{
						case 0:
							Car_Go(143);
							State_Go++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								delay_ms(400);//��ʱһ��ʱ��ʶ������
								if(LorR!=0) 
								{
									State_Go=8;//ת��״̬8
									if(LorR==1)//�����
										Target_Room='C';
									else if(LorR==2) //���ұ�
										Target_Room='D';
									Car_Go(31);//�ߵ�ʮ��·��		
								}
								else
								{
									Car_Go(86);
									State_Go++;
								}
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Little_Flag=1;
								Car_Spin(num_spin1);//תС�Ƕȿ�����
								K210_Uart_Tx_Buffer[4]=1;//��λ��Ϊתͷ�����ֵ�flag
								State_Go++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								Little_Flag=1;
								delay_ms(400);
								if(LorR!=0)
									Far_Flag=1;
								else
									Far_Flag=2;
								Car_Spin(num_spin2);
								K210_Uart_Tx_Buffer[4]=0;//��λ��Ϊתͷ�����ֵ�flag
								State_Go++;
							}
						break;
						case 4:
							if(Stop_Flag==1)
							{
								Little_Flag=0;
								Car_Go(33);
								State_Go++;
							}
						break;
						case 5:
							if(Stop_Flag==1)
							{
								Little_Flag=0;
								if(Far_Flag==1)
									Car_Spin(left_90);
								else if(Far_Flag==2)
									Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 6:
							if(Stop_Flag==1)
							{
								Car_Go(51);
								State_Go++;
							}
						break;
						case 7:
							if(Stop_Flag==1)
							{
								delay_ms(400);//��ʱһ��ʱ��ʶ������
								if(Far_Flag==1)
								{
									if(LorR==1)//�����
										Target_Room='E';
									else //���ұ�
										Target_Room='F';
								}
								else if(Far_Flag==2)
								{
									if(LorR==1)//�����
										Target_Room='H';
									else if(LorR==2) //���ұ�
										Target_Room='G';		
								}								
								Car_Go(33);
								State_Go++;
							}
						break;
						case 8:
							if(Stop_Flag==1)
							{
								if(Target_Room=='C'||Target_Room=='E'||Target_Room=='H')
									Car_Spin(left_90);
								else 
									Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 9:
							if(Stop_Flag==1)
							{
								Car_Go(33);
								State_Go++;
							}
						break;
						case 10:
							if(Stop_Flag==1)
							{
								Finish=0;
								Straight_Flag=0;
								Spin_Flag=0;
								PEout(0)=0;
							}
						break;
					}
				}
			}
		else if(Load_Flag==2)
		{
				PEout(0)=1;
				if(Target_Room=='A')
				{
					switch(State_Back)
					{
						case 0:
							Car_Go(-35);
							Back_Flag=1;
							State_Back++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								Back_Flag=0;
								Car_Spin(left_90);
								State_Back++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Car_Go(53);
								State_Back++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								Finish=0;
								PBout(9)=1;
								Straight_Flag=0;
								Spin_Flag=0;
							}
						break;
					}
				}
				else if(Target_Room=='B')
				{
					switch(State_Back)
					{
						case 0:
							Car_Go(-35);
							Back_Flag=1;
							State_Back++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								Back_Flag=0;
								Car_Spin(right_90);
								State_Back++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Car_Go(53);
								State_Back++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								Finish=0;
								Straight_Flag=0;
								Spin_Flag=0;
								PBout(9)=1;
							}
						break;
					}
				}		
				
			else if(Target_Room=='C')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=0x01;//���׼����
						Car_Go(-35);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Blooth_Uart_Tx_Buffer[4]=0x01;//С��2����
							Car_Go(142);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='D')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=0x01;//���׼����
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Blooth_Uart_Tx_Buffer[4]=0x01;//С��2����
							Car_Go(142);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Finish=0;
							Straight_Flag=0;
							Spin_Flag=0;
							PBout(9)=1;
						}
					break;
				}
			}
			else if(Target_Room=='E')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=1;//С��2����
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='F')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=1;//С��2����
						Car_Go(-35);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='G')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=1;//С��2����
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='H')
			{
				switch(State_Back)
				{
					case 0:
						Blooth_Uart_Tx_Buffer[2]=1;//С��2����
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			}
		}
	}
}
