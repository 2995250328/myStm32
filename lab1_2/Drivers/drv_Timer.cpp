#include "drv_Timer.hpp"
#include "stm32f10x.h"
#include "sys.h"
#include "GUI.hpp"
#include "GUI_Images.hpp"

extern uint16_t cnt;

void Timer_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ�ܶ�ʱ��ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=9999;//�Զ�װ��ֵ
 TIM_TimeBaseInitStrue.TIM_Prescaler=7199; //Ԥ��Ƶϵ��
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
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //��� TIM4 �����жϷ������
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update ); //��� TIM4 �����жϱ�־
		//����ÿ����ʾһ��ͼƬ
		PEout(0)=!PEout(0);
		if(cnt==0)
			cnt++;	
		else if(cnt==1)
			cnt++;
		else if(cnt==2)
			cnt++;
		else if(cnt==3)
			cnt++;
		else if(cnt==4)
			cnt++;
		else if(cnt==5)
			cnt=0;
		
	}
}
}