#include "drv_Check.hpp"
#include "delay.h"
void init_drv_Check(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //②开启 AFIO 时钟
	
	//PD14接受光耦信息，0为无 1为有
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
  GPIO_EXTILineConfig(GPIO_PortSourceGPIOD,GPIO_PinSource14);//③
	EXTI_InitStructure.EXTI_Line=EXTI_Line14;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿触发
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); //④初始化中断线参数
	
	 NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn; 
   NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // 使能
   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //抢占优先级
   NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //响应优先级
   NVIC_Init(&NVIC_InitStructure);
}
extern int Load_Flag;
extern int Delay_Flag; 
extern "C"{
void EXTI15_10_IRQHandler(void)
{
	//EXTI_ClearITPendingBit(EXTI_Line14); //手动清除外部中断标志位 
	delay_ms(5);
	PDout(15)=!PDout(15);
	if(GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_14)==1)//检测到药品
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
	EXTI_ClearITPendingBit(EXTI_Line14); //手动清除外部中断标志位 
}
}