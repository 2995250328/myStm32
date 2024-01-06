#include "stm32f10x.h"
#include "drv_LED.hpp"
#include "Basic.hpp"
#include "delay.h"

#define APB1TIMERCLK 36000000

float ExtLedR = 0;
float ExtLedG = 0;
float ExtLedB = 0;


//�������Բ�������
static inline float led_linear_compensation( float in )
{
	if( in <= 8 )
		return in / 903.3f * 100;
	else
		return 0;
}

/*
	LED�����Ⱥ���
	R��G��B�����Ȱٷֱȣ�0-100��
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
	������Ƶ�ʵ��ں���
	freq:������Ƶ��
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
	���������캯��
	on:�Ƿ�����
*/
void set_BuzzerOnOff( bool on )
{
	if(on)
		TIM5->CCR4 = TIM5->ARR / 2;
	else
		TIM5->CCR4 = 0;
}


//����LEDģʽ
void Set_LED_Mode(LED_Mode mode)
{
	led_status = mode;
}

//LED��˸����
void Led_Flash()
{
	GPIO_ResetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
	GPIO_SetBits(GPIOE,GPIO_Pin_0);
	delay_ms(500);
}

//LED��ʼ������
void init_drv_LED(void)
{
	/*
		D3_LED(GPIOE_P0)  PE0
		LED_B(TIM5_CH1)   PA0
		LED_G(TIM5_CH2)   PA1
		LED_R(TIM5_CH3)   PA2
		Buzzer(TIM5_CH4)  PA3
	*/
	//����GPIOE��GPIOA����ʱ��,�������ù���ʱ��
	RCC->APB2ENR|=(1<<6)|(1<<2)|(1<<0);
	//D3��ʼ��
	os_delay(1e-2);
	//�����������ģʽ
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);	    //ʹ��PE�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;			    	//D3-->PE.0 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOE, &GPIO_InitStructure);			     		//��ʼ��GPIOE.0
	GPIO_SetBits(GPIOE,GPIO_Pin_5);								//PE.0 �����
	PEout(0)=1;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;			    	//PB9 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     		//��ʼ��GPIOD15
	GPIO_SetBits(GPIOB,GPIO_Pin_9);								//PB9 �����
	PBout(9)=0;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	    //ʹ��PB�˿�ʱ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;			    	//PB8 �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 	 		//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	 		//IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);			     		//��ʼ��GPIOD15
	GPIO_SetBits(GPIOB,GPIO_Pin_8);								//PB8 �����
	PBout(8)=0;
	
	//RGB Buzzer��ʼ��
	//�������Ÿ���ģʽ
	set_register( GPIOA->CRL , 0b11 , 0 , 2 );	//PA0
	set_register( GPIOA->CRL , 0b11 , 4 , 2 );	//PA1
	set_register( GPIOA->CRL , 0b11 , 8 , 2 );	//PA2
	set_register( GPIOA->CRL , 0b11 , 12 , 2 );	//PA3
	//���������죬LED��©
	set_register( GPIOA->CRL , 0b11 , 2 , 2 );	//PA6
	set_register( GPIOA->CRL , 0b11 , 6 , 2 );	//PA7
	set_register( GPIOA->CRL , 0b11 , 10 , 2 );	//PB0
	set_register( GPIOA->CRL , 0b10 , 14 , 2 );	//PB1
	//����TIM5����ʱ��
	RCC->APB1ENR|=(1<<3);
	os_delay(1e-2);
	//������Ƶ��10mhz
	TIM5->PSC = (APB1TIMERCLK / 3600000) - 1;
	//�趨��������װ��ֵ
	TIM5->ARR = 10e6 / 1000;
	//����PWM1ģʽ
	//ͨ��һ��110��PWMģʽ1����ֻҪ TIMx_CNT<TIMx_CCRx��ͨ��1��Ϊ��Ч״̬
	//ͨ������111��PWMģʽ2����ֻҪ TIMx_CNT<TIMx_CCRx��ͨ��1��Ϊ��Ч״̬
	//����Ƚ� 1234 Ԥװ��ʹ��
	//CC 1234 ͨ������Ϊ���
	TIM5->CCMR1 = (0b111<<12)|(1<<11)|(0<<8) | (0b110<<4)|(1<<3)|(0<<0);
	TIM5->CCMR2 = (0b111<<12)|(1<<11)|(0<<8) | (0b111<<4)|(1<<3)|(0<<0);
	//��λ���
	TIM5->CCR1 = TIM5->CCR2 = TIM5->CCR3 = TIM5->CCR4 = 0;
	//������ʱ��
	//����Ӧ������������ OC1 OC2 OC3 OC4 �ź�
	TIM5->CCER = (1<<12) | (1<<8) | (1<<4) | (1<<0);
	//ʹ�ܶ�ʱ��
	TIM5->CR1 = (1<<7)|(1<<0);
}