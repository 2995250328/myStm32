#include "drv_Check.hpp"
#include "delay.h"
void init_drv_Check(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //�ڿ��� AFIO ʱ��
	
	//PD14���ܹ�����Ϣ��0Ϊ�� 1Ϊ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource14);//��
	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //�����ش���
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); //�ܳ�ʼ���ж��߲���
	
	 NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn; 
   NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // ʹ��
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //��ռ���ȼ�
   NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //��Ӧ���ȼ�
   NVIC_Init(&NVIC_InitStructure);
}
extern int Load_Flag;
extern int Delay_Flag; 
extern "C"{
void EXTI15_10_IRQHandler(void)
{
	//EXTI_ClearITPendingBit(EXTI_Line14); //�ֶ�����ⲿ�жϱ�־λ 
	delay_ms(5);
	PDout(15)=!PDout(15);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14)==1)//��⵽ҩƷ
	{
	
		if(Load_Flag==0)
			Load_Flag=1;
		Delay_Flag=1;
	}
	else
	{
		if(Load_Flag==1)
			Load_Flag=2;
		Delay_Flag=1;
	}
	EXTI_ClearITPendingBit(EXTI_Line14); //�ֶ�����ⲿ�жϱ�־λ 
}
}