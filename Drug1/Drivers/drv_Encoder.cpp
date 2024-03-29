#include "drv_Encoder.hpp"
#include "stm32f10x_gpio.h"
#include "usart.h"
#include "Control.hpp"	
/*
 * 函数功能：把TIM3初始化为编码器接口模式
 * 入口参数：无
 * 返回  值：无
 */
void drv_Encoder_Init_TIM3(void)
{
	// PC6  ---> E1A  T3C1
	// PC7  ---> E1B  T3C2
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	//重映射使能
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3,ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);					      
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
	//设定计数器自动重装值
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; //65535
	//选择时钟分频：不分频
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	//TIM向上计数  
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	//使用编码器模式3
  TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
	//滤波10
  TIM_ICInitStructure.TIM_ICFilter = 10;	
  TIM_ICInit(TIM3, &TIM_ICInitStructure);
	//清除TIM的更新标志位
  TIM_ClearFlag(TIM3, TIM_FLAG_Update);
  TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM3,0);
  TIM_Cmd(TIM3, ENABLE); 
}

/*
 * 函数功能：把TIM4初始化为编码器接口模式
 * 入口参数：无
 * 返回  值：无
 */
void drv_Encoder_Init_TIM4(void)
{
	// PD12 ---> E2A  T4C1
	// PD13 ---> E2B  T4C2
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
  TIM_ICInitTypeDef TIM_ICInitStructure;  
  GPIO_InitTypeDef GPIO_InitStructure;
	//使能定时器4的时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	//使能PB端口时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//重映射使能
	GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13;	
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; 
  GPIO_Init(GPIOD, &GPIO_InitStructure);					      
  
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = 0x0; 
  TIM_TimeBaseStructure.TIM_Period = ENCODER_TIM_PERIOD; 
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
  TIM_ICStructInit(&TIM_ICInitStructure);
  TIM_ICInitStructure.TIM_ICFilter = 10;
  TIM_ICInit(TIM4, &TIM_ICInitStructure);
  TIM_ClearFlag(TIM4, TIM_FLAG_Update);
  TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
  //Reset counter
  TIM_SetCounter(TIM4,0);
  TIM_Cmd(TIM4, ENABLE); 
}
/*
 * 函数功能：单位时间读取编码器计数
 * 入口参数：TIMX：定时器
 * 返回  值：速度值
 */
int Read_Encoder(u8 TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
	  case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;	
		default: Encoder_TIM=0;
	 }
		return Encoder_TIM;
}
/*
 * 函数功能：TIM4中断服务函数
 * 入口参数：无
 * 返回  值：无
 */
extern "C" void TIM4_IRQHandler(void)
{ 		    		  		
	//溢出中断	
	if(TIM4->SR&0X0001)
	{    			
	}				   
	//清除中断标志位 
	TIM4->SR&=~(1<<0);	    
}
/*
函数功能：TIM3中断服务函数
入口参数：无
返回  值：无
**************************************************************************/
extern "C" void TIM3_IRQHandler(void)
{ 		    		  			    
	if(TIM3->SR&0X0001)
	{    				   			
	}				   
	TIM3->SR&=~(1<<0);	    
}
