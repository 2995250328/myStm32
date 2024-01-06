#include "sys.h"
#include "stm32f10x_exti.h"
#include "delay.h"
#include "usart.h"
#include "led.c"
#include "inv_mpu.h"
#include "GUI.hpp"
#include "GUI_Images.hpp"
#include "drv_Main.hpp"
#include "drv_LED.hpp"
#include "drv_LCD.hpp"
#include "drv_Timer4.hpp"
#include "drv_Beep.hpp"
#include "drv_Key.hpp"
#include "drv_ADC.hpp"
#include "drv_MPU6050.hpp"
#include "drv_Uart2.hpp"

extern int offset_flag;
u32 bound=115200;//�����ʱ���
u8 t;//��Ϊforѭ���ı���
u8 len;//��¼��ȡ���ݵĳ���
int num;
//�¶ȱ���
int Temperature;
//Z����ٶȼ�  
float Acceleration_Z; 	
//ƽ����� ƽ�������� ת��������
float Angle_Balance,Gyro_Balance,Gyro_Turn;
//��ʱ�������� ÿ���ж�+1 1s����һ ���������ʾˢ��
extern int time_cnt;
extern float gyro_offset[3];
extern float GYRO[3],ACC[3];
float volt=0;//�洢��ѹֵ�ı���
u16 n=0;
u8 OFF[]={"Solving static bias,don't move"};

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
	//��ʼ����ʱ��TIM4
	Timer4_init(999,359);//  72m/1000/360=200hz  0.005s
	//��ʼ����ĻΪ��ɫ
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	//ADC1�ĳ�ʼ��
	init_drv_ADC();
	//�����жϳ�ʼ��
	EXTIX_Init();
	/*MPU6050��ʼ��*/
	MPU_Init();
	//�ȴ�MPU6050��ʼ�����
	while(mpu_dmp_init());
	//��ʼ��uart1
	uart_init(115200);
	init_drv_Uart2(9600);


	//������ѭ��
	while(1)
	{	
		//ʵ�����ݻش�
		if(USART_RX_STA&0x8000)
		{
			len=USART_RX_STA&0x3f; //�õ��˴ν��յ������ݳ���
			printf("\r\nthe massage is:\r\n");
			for(t=0;t<len;t++)
			{ 
				USART_SendData(USART1, USART_RX_BUF[t]); //�򴮿� 1 ��������
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
				//�ȴ����ͽ���
			}
			printf("\r\n"); //���뻻��
			USART_RX_STA=0;
		}
		if(offset_flag==0)
			LCD_ShowString(0,128,OFF,RED,BLACK,16,0);
		else if(offset_flag==2)
			LCD_Fill(0,0,LCD_W,LCD_H,RED);
		//��ʾ��Ҫ����
		if((time_cnt%20)==0&&offset_flag==1)
		{
			LCD_Fill(0,128,256,144,WHITE);
			LCD_ShowMAIN();	
		}
	}
}
