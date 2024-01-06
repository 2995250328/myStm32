#include "stm32f10x.h"
#include "drv_LED.hpp"
#include "Basic.hpp"
#include "delay.h"

#define APB1TIMERCLK 36000000

float ExtLedR = 0;
float ExtLedG = 0;
float ExtLedB = 0;


//亮度线性补偿函数
static inline float led_linear_compensation( float in )
{
	if( in <= 8 )
		return in / 903.3f * 100;
	else
		return 0;
}

/*
	LED调亮度函数
	R、G、B：亮度百分比（0-100）
*/
void set_LedBrightness( float R , float G , float B )
{
	if( B >= 0 && B <= 100 )
	{
		B = led_linear_compensation(B);
		ExtLedB = B;
		TIM5->CCR1 = B/100*TIM5->ARR;
	}
	if( G >= 0 && G <= 100 )
	{
		G = led_linear_compensation(G);
		ExtLedG = G;
		TIM5->CCR2 = G/100*TIM5->ARR;
	}
	if( R >= 0 && R <= 100 )
	{
		R = led_linear_compensation(R);
		ExtLedR = R;
		TIM5->CCR3 = R/100*TIM5->ARR;
	}
}

static LED_Mode led_status = normal;

/*
	蜂鸣器频率调节函数
	freq:蜂鸣器频率
*/
void set_BuzzerFreq( unsigned short freq )
{
	float B = (float)TIM5->CCR1/(float)TIM5->ARR;
	float G = (float)TIM5->CCR2/(float)TIM5->ARR;
	float R = (float)TIM5->CCR3/(float)TIM5->ARR;
	if( freq < 200 )
		freq = 200;
	TIM5->ARR = 10e6 / freq;
	if( TIM5->CCR4 != 0 )
		TIM5->CCR4 = TIM5->ARR / 2;
	TIM5->CCR1 = B*TIM5->ARR;
	TIM5->CCR2 = G*TIM5->ARR;
	TIM5->CCR3 = R*TIM5->ARR;
}

/*
	蜂鸣器鸣响函数
	on:是否鸣响
*/
void set_BuzzerOnOff( bool on )
{
	if(on)
		TIM5->CCR4 = TIM5->ARR / 2;
	else
		TIM5->CCR4 = 0;
}


//设置LED模式
void Set_LED_Mode(LED_Mode mode)
{
	led_status = mode;
}

//LED闪烁函数
void Led_Flash()
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
	GPIO_SetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
}

//LED初始化函数
void init_drv_LED(void)
{
	/*
		D3_LED(GPIOE_P0)  PE0
		LED_B(TIM5_CH1)   PA0
		LED_G(TIM5_CH2)   PA1
		LED_R(TIM5_CH3)   PA2
		Buzzer(TIM5_CH4)  PA3
	*/
	//开启GPIOE、GPIOA外设时钟,开启复用功能时钟
	RCC->APB2ENR|=(1<<6)|(1<<2)|(1<<0);
	//D3初始化
	os_delay(1e-2);
	//配置引脚输出模式
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	    //使能PE端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			    	//D3-->PE.0 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO口速度为50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);			     		//初始化GPIOE.0
	GPIO_SetBits(GPIOE,GPIO_Pin_5);								//PE.0 输出高
	PEout(0)=1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			    	//PB9 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     		//初始化GPIOD15
	GPIO_SetBits(GPIOB,GPIO_Pin_9);								//PB9 输出高
	PBout(9)=0;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //使能PB端口时钟
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			    	//PB8 端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO口速度为50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     		//初始化GPIOD15
	GPIO_SetBits(GPIOB,GPIO_Pin_8);								//PB8 输出高
	PBout(8)=0;
	
	//RGB Buzzer初始化
	//配置引脚复用模式
	set_register( GPIOA->CRL , 0b11 , 0 , 2 );	//PA0
	set_register( GPIOA->CRL , 0b11 , 4 , 2 );	//PA1
	set_register( GPIOA->CRL , 0b11 , 8 , 2 );	//PA2
	set_register( GPIOA->CRL , 0b11 , 12 , 2 );	//PA3
	//蜂鸣器推挽，LED开漏
	set_register( GPIOA->CRL , 0b11 , 2 , 2 );	//PA6
	set_register( GPIOA->CRL , 0b11 , 6 , 2 );	//PA7
	set_register( GPIOA->CRL , 0b11 , 10 , 2 );	//PB0
	set_register( GPIOA->CRL , 0b10 , 14 , 2 );	//PB1
	//开启TIM5外设时钟
	RCC->APB1ENR|=(1<<3);
	os_delay(1e-2);
	//计数器频率10mhz
	TIM5->PSC = (APB1TIMERCLK / 3600000) - 1;
	//设定计数器重装载值
	TIM5->ARR = 10e6 / 1000;
	//配置PWM1模式
	//通道一三110：PWM模式1——只要 TIMx_CNT<TIMx_CCRx，通道1便为有效状态
	//通道二四111：PWM模式2——只要 TIMx_CNT<TIMx_CCRx，通道1便为无效状态
	//输出比较 1234 预装载使能
	//CC 1234 通道配置为输出
	TIM5->CCMR1 = (0b111<<12)|(1<<11)|(0<<8) | (0b110<<4)|(1<<3)|(0<<0);
	TIM5->CCMR2 = (0b111<<12)|(1<<11)|(0<<8) | (0b111<<4)|(1<<3)|(0<<0);
	//复位输出
	TIM5->CCR1 = TIM5->CCR2 = TIM5->CCR3 = TIM5->CCR4 = 0;
	//开启定时器
	//在相应输出引脚上输出 OC1 OC2 OC3 OC4 信号
	TIM5->CCER = (1<<12) | (1<<8) | (1<<4) | (1<<0);
	//使能定时器
	TIM5->CR1 = (1<<7)|(1<<0);
}