#include "drv_Timer.hpp"
#include "stm32f10x.h"
#include "sys.h"
#include "GUI.hpp"
#include "GUI_Images.hpp"

extern uint16_t cnt;

void Timer_init(void)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能定时器时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=9999;//自动装载值
 TIM_TimeBaseInitStrue.TIM_Prescaler=7199; //预分频系数
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
void TIM4_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //检查 TIM4 更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update ); //清除 TIM4 更新中断标志
		//控制每秒显示一张图片
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