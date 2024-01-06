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


char operation='+';//����������
int operative_index=1;//�������任������
int number1[3]={0};//�����洢��һ������
int number2[3]={0};//�����洢�ڶ�������
int indication=0;//��Ϊ����λ�л���ָʾ
u16 result=0;
int show1=1,show2=1,show3=1;//����ͼƬ����ʾ��־
int num=0;
u8 str[]={"indication:"};
uint16_t cnt=0;//���Ա���

u16 n=0;

//�캣���
u8 music1[]={5,5,6,8,7,6,5,6,13,13,5,5,6,8,7,6,5,3,13,13,2,2,3,5,3,5,6,3,2,1,6,6,5,6,5,3,6,5,13,13,
                  5,5,6,8,7,6,5,6,13,13,5,5,6,8,7,6,5,3,13,13,2,2,3,5,3,5,6,3,2,1,6,6,5,6,5,3,6,1,    
              13,8,9,10,10,9,8,10,9,8,6,13,6,8,9,9,8,6,9,8,6,5,13,2,3,5,5,3,5,5,6,8,7,6,6,10,9,9,8,6,5,6,8  };            
u8 time1[] = {2,4,2,2,2,2,2,8,4, 4,2,4,2,2,2,2,2,8,4, 4, 2,4,2,4,2,2,4,2,2,8,2,4,2,2,2,2,2,8,4 ,4, 
                2,4,2,2,2,2,2,8,4, 4,2,4,2,2,2,2,2,8,4,4,2,4,2,4,2,2,4,2,2,8,2,4,2,2,2,2,2,8,
                4,2,2,2,4,2,2,2,2,2,8,4,2,2,2,4,2,2,2,2,2,8,4,2,2,2,4,2,2,5,2,6,2,4,2,2,2,4,2,4,2,2,12  };
//���տ���
u8 music2[]={5,5,6,5,8,7,5,5,6,5,9,8,5,5,12,10,8,7,6,11,
                  11,10,8,9,8,5,5,8,5,5,12,10,8,7,6,11,11,10,8,9,8    //��������
    };     
        u8 time2[] = {1,2,2,2,2,4,1,2,2,2,2,4,1,2,2,2,1,4,
                      4,1,2,2,2,2,4,1,2,4,1,2,2,2,1,4, 4,1,2,2,2,2,4,4        //����ʱ��
        }; 
//С����
u8 music3[]= {3,5,8,6,5,13, //����
             3,5,6,8,5,13,
             8,10,9,8,9,8,6,8,5,13,
             3,5,6,5,6,8,9,5,6,13,
             3,2,1,2,13,
             2,2,3,5,5,8,2,3,5,13
            };
u8 time3[] = {2,2,2,2,6,4, //ʱ��
             2,2,2,2,6,4,
             6,2,4,4,2,2,2,2,6,4,
             6,2,4,2,2,4,2,2,6,4,
             2,2,4,6,4,
             4,2,2,4,4,4,2,2,6,4
            };

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
	BEEP_Init();
	//�����жϳ�ʼ��
	EXTIX_Init();
	//init_drv_Key();

	//������ѭ��
	while(1)
	{
//		//����1 �ö�ʱ��ÿ���л�һ��ͼƬ
//		if(cnt==0)
//			LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
//		else if(cnt==1)
//			LCD_ShowPicture(0,0,40,40,gImage_ex3);
//		else if(cnt==2){
//			LCD_ShowPicture(0,0,80,80,gImage_ex2);}
//		else if(cnt==3){
//			LCD_ShowPicture(0,0,80,80,gImage_ex1);}
//		else if(cnt==4){
//			LCD_ShowPicture(0,0,80,80,gImage_ex4);	}
//		else if(cnt==5){
//			LCD_ShowPicture(0,0,240,240,gImage_ex5);}
		//����2 ��ʾ������
		Key();
		LCD_ShowCalculator(0,120,number1,operation,number2,result,indication);
		LCD_ShowIntNum(0,0,num,4,BLACK,WHITE,16);
		//����3 �����л�ͼƬ������
//		if(num==0)
//		{
//			show3=1;
//			if(show1==1)
//			{
//				LCD_ShowPicture(0,0,80,80,gImage_ex2);	
//				show1=0;
//			}		
//			play_music(music1,time1,sizeof(music1)/sizeof(music1[0]));
//		}
//		else if(num==1)
//		{
//			show1=1;
//			if(show2==1)
//			{
//				LCD_ShowPicture(0,0,80,80,gImage_ex1);	
//				show2=0;
//			}		
//			play_music(music2,time2,sizeof(music2)/sizeof(music2[0]));
//		}
//		else if(num==2)
//		{
//			show2=1;
//			if(show3==1)
//			{
//				LCD_ShowPicture(0,0,80,80,gImage_ex4);	
//				show3=0;
//			}		
//			play_music(music3,time3,sizeof(music3)/sizeof(music3[0]));
		}
	}

