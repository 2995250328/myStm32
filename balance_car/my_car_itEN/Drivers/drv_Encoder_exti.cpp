#include "drv_Encoder_exti.hpp"

int counter_left=0,counter_right=0;

//右轮外部中断读编码器
void Encoder_Init_Exit_Right(void)
{
	// PC6  ---> E1A  T3C1
	// PC7  ---> E1B  T3C2   
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
 
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI9_5_IRQn; //选择触发的通道 EXTI9_5_IRQn 与 PC7 EXTI_Line7 一致
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // 使能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //响应优先级
    NVIC_Init(&NVIC_InitStructure);
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;// 浮空输入
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7|GPIO_PinSource6); //配置AFIO的数据选择器，来选择我们要的引脚，指定外部中断线
 
    EXTI_InitStructure.EXTI_Line    = EXTI_Line7|EXTI_Line6;                  //需要我们配置中断线
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                      //开启中断
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;            //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置触发方式 上升下降沿触发
    EXTI_Init(&EXTI_InitStructure);                                //调用这个参数，就可以根据这个结构体
}

extern "C" {
void EXTI9_5_IRQHandler(void)    //编码器1的外部中断
{
	EXTI_ClearITPendingBit(EXTI_Line7); //手动清除外部中断标志位 
	EXTI_ClearITPendingBit(EXTI_Line6); //手动清除外部中断标志位 
	if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //pc6上升沿
	{	
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //判断 encoder_pin_B 电平状态，
    {
			counter_right--;
    }    
    else
    {
			counter_right++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6))//pc6下降沿
	{
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //判断 encoder_pin_B 电平状态，
    {
			counter_right--;
    }    
    else
    {
			counter_right++;
    }
	}
	if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //pc7上升沿
	{	
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //判断 encoder_pin_A 电平状态，
    {
			counter_right++;
    }    
    else
    {
			counter_right--;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))//pc7下降沿
	{
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //判断 encoder_pin_A 电平状态，
    {
			counter_right++;
    }    
    else
    {
			counter_right--;
    }
	}
}
}
int Encoder_Readnum_Right(void)
{		
    int result = counter_right;
		counter_right=0;
		return result;
}
//左轮编码器外部中断读取
void Encoder_Init_Exit_Left(void)
{
		// PD12 ---> E2A  T4C1
		// PD13 ---> E2B  T4C2
		GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
 
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn; //选择触发的通道 EXTI15_10_IRQn 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // 使能
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //响应优先级
    NVIC_Init(&NVIC_InitStructure);
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;// 浮空输入
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
   
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12|GPIO_PinSource13); //配置AFIO的数据选择器，来选择我们要的引脚，指定外部中断线
 
    EXTI_InitStructure.EXTI_Line    = EXTI_Line12|EXTI_Line13;                  //需要我们配置中断线
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                      //开启中断
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;            //中断模式
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //设置触发方式 上升下降沿触发
    EXTI_Init(&EXTI_InitStructure);                                //调用这个参数，就可以根据这个结构体
}
extern "C" {
void EXTI15_10_IRQHandler(void)    //编码器2的外部中断
{	
	if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)) //pd12上升沿
	{	
		if (0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //判断 encoder_pin_B 电平状态，
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))//pd12下降沿
	{
		if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //判断 encoder_pin_B 电平状态，
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //pd13上升沿
	{	
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)) //判断 encoder_pin_A 电平状态，
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))//pd13下降沿
	{
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)) //判断 encoder_pin_A 电平状态，
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
		EXTI_ClearITPendingBit(EXTI_Line12); //手动清除外部中断标志位 
	EXTI_ClearITPendingBit(EXTI_Line13); //手动清除外部中断标志位 
}
}
int Encoder_Readnum_left(void)
{		
    int result = counter_left;
		counter_left=0;
		return result;
}