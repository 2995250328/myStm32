#include "drv_Timer4.hpp"
#include "drv_MPU6050.hpp"
#include "drv_ADC.hpp"
#include "drv_Key.hpp"
#include "stm32f10x.h"
#include "sys.h"
#include "inv_mpu.h"
#include "usart.h"
#include "filter.h"
#include <math.h>

/*
	�������ã���ʱ����ʼ��
	psc��Ԥ��Ƶϵ��
	arr���Զ�װ��ֵ
	Tout= ((arr+1)*(psc+1))/Tclk��
*/
void Timer4_init(int psc,int arr)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ�ܶ�ʱ��ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=psc;//�Զ�װ��ֵ
  TIM_TimeBaseInitStrue.TIM_Prescaler=arr; //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//��ʼ��
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//������ʱ���ж� �����жϵ��������ʱ�����ж�,����ָ���Ǹ��¼�������ֵ,�ж���ָ������ֵ��ʱ�������ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // ����NVIC�жϷ���2,����2λ��ռ���ȼ���2λ��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;//�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;//��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//ʹ��
  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC
	TIM_Cmd(TIM4,ENABLE);//ʹ�ܶ�ʱ��
}
extern "C"{
void TIM4_IRQHandler(void)//200Hz 0.005s
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //��� TIM4 �����жϷ������
	{
		
		
	}
}
}