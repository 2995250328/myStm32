#include "drv_Timer2.hpp"
#include "drv_ADC.hpp"
#include "drv_Encoder.hpp"
#include "usart.h"
#include "Control.hpp"
#include "inv_mpu.h"
#include "stm32f10x.h"
#include "sys.h"
void Timer2_init()// 72M/1000/360=200hz  0.005s
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

int Voltage_temp;//�洢ÿһ����ʱ���жϻ�õĵ�ѹֵ
int Voltage_sum=0;//�洢100����ʱ���жϻ�õ��ܵ�ѹֵ���Ӷ�ȡƽ��
int time_cnt=0;//�����жϴ����ļ�������
int Volt_flag=0;//��ѹֵС��11.7��flag
extern float Voltage; 
extern float Pitch,Roll,Yaw;
extern int Encoder_Left,Encoder_Right;

extern "C" void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{    
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //��� TIM2 �����жϱ�־		
		if(time_cnt<100)//ÿһ�ٴμ�0.5s����һ�ε�ѹֵ
		{
			Voltage_temp=Get_battery_volt();
			Voltage_sum+=Voltage_temp;
			time_cnt++;
		}
		else if(time_cnt==100)
		{
			time_cnt=0;
			Voltage=(float)Voltage_sum/100;
			Voltage_sum=0;//���� ��ֹӰ�����
			if(Voltage/100<11.7&&Volt_flag==0)
				Volt_flag=1;
			else
				Volt_flag=0;
			
			PEout(0)=!PEout(0);//������100 ��0.5s ����LED��˸ �Ա�ʾ��������
		}
		//��ȡ���ٶȡ����ٶȡ����(������ת)
		mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
		//��ȡ������
		Encoder_Left=-Read_Encoder(4);
		Encoder_Right=Read_Encoder(3);
		#if Debug
			#if Debug_pos
   					Motor_Left=Position_PID(Encoder_Left,Target_Position_L);
				  	Motor_Left=PWM_Limit(Motor_Left,6700,-6700);
//						Motor_Right=Position_PID(Encoder_Right,Target_Position_R);
//    				Motor_Right=PWM_Limit(Motor_Right,6700,-6700);
					  Set_Pwm(Motor_Left,0);
   		#elif Debug_vel
		
			#endif
		#endif
	}				       
}