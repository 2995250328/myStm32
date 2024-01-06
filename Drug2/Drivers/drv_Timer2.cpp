#include <math.h>
#include "drv_Timer2.hpp"
#include "drv_ADC.hpp"
#include "drv_Encoder.hpp"
#include "drv_MPU6050.hpp"
#include "drv_OpenMv.hpp"
#include "drv_K210.hpp"
#include "drv_Blooth.hpp"
#include "filter.h"
#include "usart.h"
#include "Control.hpp"
#include "inv_mpu.h"
#include "stm32f10x.h"
#include "sys.h"
void Timer2_init()// 72M/1000/360=200hz 
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//ʹ�ܶ�ʱ��ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=999;//�Զ�װ��ֵ
  TIM_TimeBaseInitStrue.TIM_Prescaler=359; //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrue);//��ʼ��
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //��� TIM2 �����жϱ�־		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//������ʱ���ж� �����жϵ��������ʱ�����ж�,����ָ���Ǹ��¼�������ֵ,�ж���ָ������ֵ��ʱ�������ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // ����NVIC�жϷ���,����0λ��ռ���ȼ���0λ��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;//�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;//��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//ʹ��
  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC
	TIM_Cmd(TIM2,ENABLE);//ʹ�ܶ�ʱ��
}
int Speed_Max=15;              //����ٶ�
int time_cnt=0;//�����жϴ����ļ�������
int delay_count=0;//���ο��Ƽ����ʱ
int Spin_Succeed[2]={0};
extern int Turn_Pwm;
extern int Encoder_Left,Encoder_Right;
extern int Position_L,Position_R;//�������ܼ���
//����������ת��Ϊʵ�ʾ���
extern int Location_CM_L,Location_CM_R;
//�ٶ�λ��pid���Ʊ���
extern int LV_PWM_L,LV_PWM_R;
extern int Delay_Transmit;
extern u8 K210_Rx_Data[8];
extern u8 OpenMv_Rx_Data[8];
extern int Back_Flag;
extern int Little_Flag;
extern int Finish;
extern "C" void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{    
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //��� TIM2 �����жϱ�־	
		if(time_cnt<60)
			time_cnt++;
		else if(time_cnt==60)		
			K210_Check_Data_Task();
		Blooth_Check_Data_Task();
		OpenMv_Check_Data_Task();
		//��ȡ������
		Encoder_Left=Read_Encoder(4);
		Encoder_Right=Read_Encoder(3);
		//����������ת��Ϊ�ٶ�
		Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
		//�ܱ���������
		Position_L+=Encoder_Left;
		Position_R+=Encoder_Right;
		//����������ת��Ϊʵ�ʾ���
		Location_CM_L=((float)Position_L/1560)*((float)Perimeter/10);
		Location_CM_R=((float)Position_R/1560)*((float)Perimeter/10);
		if(Straight_Flag==1)//ֱ�еĿ���
		{
			Turn_Pwm=OPENMV_Turn_PID(OpenMv_Rx_Data[0]-100);
			if((Position_L+Position_R)/2<(Target_Position_L+150)&&(Position_L+Position_R)/2>(Target_Position_L-150))
			{
				delay_count++;
				LV_PWM_L=Location_Speed_L(Encoder_Left,Position_L,Target_Position_L,0);
				LV_PWM_R=Location_Speed_R(Encoder_Right,Position_R,Target_Position_R,0);
				if(delay_count>=30)
				{
					Straight_Flag=0;
					Stop_Flag=1;
					delay_count=0;
				}
			}
			else 
			{
				LV_PWM_L=Location_Speed_L(Encoder_Left,Position_L,Target_Position_L,Speed_Max);
				LV_PWM_R=Location_Speed_R(Encoder_Right,Position_R,Target_Position_R,Speed_Max);
				Stop_Flag = 0;  
				delay_count = 0; 
			}
			if(Back_Flag==0)
			{
				Motor_Left=LV_PWM_L+Turn_Pwm;
				Motor_Right=LV_PWM_R-Turn_Pwm;
			}				
			else
			{
				Motor_Left=LV_PWM_L;
				Motor_Right=LV_PWM_R;
			}
		}
		if(Spin_Flag==1)//ת�����
		{
			if(Position_L<(Target_Position_L+40)&&Position_L>(Target_Position_L-40))
			{
				Spin_Succeed[0]=1;
				LV_PWM_L=Location_Speed_L(Encoder_Left,Position_L,Target_Position_L,0);
			}
			else
			{
				Spin_Succeed[0]=0;
				if(Little_Flag==0)
					LV_PWM_L=Location_Speed_L(Encoder_Left,Position_L,Target_Position_L,Speed_Max*0.5);
				else
					LV_PWM_L=Location_Speed_L(Encoder_Left,Position_L,Target_Position_L,Speed_Max*0.27);
			}
			if(Position_R<(Target_Position_R+40)&&Position_R>(Target_Position_R-40))
			{
				Spin_Succeed[1]=1;
				LV_PWM_R=Location_Speed_R(Encoder_Right,Position_R,Target_Position_R,0);
			}
			else 
			{	
				Spin_Succeed[1]=0;
				if(Little_Flag==0)
					LV_PWM_R=Location_Speed_R(Encoder_Right,Position_R,Target_Position_R,Speed_Max*0.5);
				else
					LV_PWM_R=Location_Speed_R(Encoder_Right,Position_R,Target_Position_R,Speed_Max*0.27);
			}
			if(Spin_Succeed[0]==1&&Spin_Succeed[1]==1)
			{
				delay_count++;
				if(delay_count>=20)
				{
					Spin_Succeed[0]=0;
					Spin_Succeed[1]=0;
					Stop_Flag = 1;  
					Spin_Flag=0;
					delay_count = 0; 
				}
			}
			Motor_Left=LV_PWM_L;
			Motor_Right=LV_PWM_R;
		}
		if(Spin_Flag==0&&Straight_Flag==0)
		{
			Motor_Left=0;
			Motor_Right=0;
		}
		Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
		Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM�޷�		
		Set_Pwm(Motor_Left,Motor_Right);	
		
	}				       
}