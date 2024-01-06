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
	函数作用：定时器初始化
	psc：预分频系数
	arr：自动装载值
	Tout= ((arr+1)*(psc+1))/Tclk；
*/
void Timer4_init(int psc,int arr)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能定时器时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=psc;//自动装载值
  TIM_TimeBaseInitStrue.TIM_Prescaler=arr; //预分频系数
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//初始化
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启定时器中断 更新中断当计数溢出时进入中断,更新指的是更新计数器的值,中断是指更新数值的时候会进入中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 设置NVIC中断分组2,其中2位抢占优先级，2位响应优先级
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;//中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 2;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//使能
  NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	TIM_Cmd(TIM4,ENABLE);//使能定时器
}
extern "C"{
void TIM4_IRQHandler(void)//200Hz 0.005s
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //检查 TIM4 更新中断发生与否
	{
		
		
	}
}
}