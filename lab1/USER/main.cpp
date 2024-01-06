#include "sys.h"
#include "stm32f10x_exti.h"
#include "delay.h"
#include "usart.h"
#include "led.c"
#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "Commulink.hpp"
#include "GUI.hpp"
#include "GUI_Images.hpp"
#include "drv_Timer.hpp"
#include "drv_Beep.hpp"
#include "drv_Key.hpp"

int len; int t;
char operation='+';//����������
int operative_index=1;//�������任������
int number1[3]={0};//�����洢��һ������
int number2[3]={0};//�����洢�ڶ�������
int indication=0;//��Ϊ����λ�л���ָʾ
float result=0;
int show1=1,show2=1,show3=1;//����ͼƬ����ʾ��־
int num=0;
u8 str[]={"indication:"};
uint16_t cnt=0;//���Ա���

u16 n=0;

RCC_ClocksTypeDef RCC_CLK;
int main(void)
{	
	//ϵͳʱ�ӳ�ʼ��
	SystemInit();
	//��ʱʱ�ӳ�ʼ��
  delay_init();	       
	//��ȡʱ������
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	
	//��ʼ���豸����
	init_drv_Main();
	//LED��ʼ��
	LED_Init();
	//��ʼ����ʱ��TIM4
	Timer_init();
	//��ʼ����ĻΪ��ɫ
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	//��ʼ��������
//	BEEP_Init();
	//�����жϳ�ʼ��
	EXTIX_Init();
	//init_drv_Key();
	u32 bound=115200;//�����ʱ���
	uart_init(115200);
	//������ѭ��
	while(1)
	{
		if(USART_RX_STA&0x8000)
 { len=USART_RX_STA&0x3f; //�õ��˴ν��յ������ݳ���
printf("\r\n message:\r\n\r\n");
for(t=0;t<len;t++)
{ USART_SendData(USART1, USART_RX_BUF[t]); //�򴮿� 1 ��������
 while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
//�ȴ����ͽ���
}
printf("\r\n\r\n"); //���뻻��
USART_RX_STA=0;}
	}
}
