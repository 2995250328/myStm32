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

float last_gyrox=0;//��¼�ϴε�����������
int nn=0;
int offset_flag=0;
float sum1=0,sum2=0,sum3=0;//���
int cnt=0;
extern float gyro_offset[3];
int i=0;
extern int flag;
extern int way;//�����㷨�ı���
extern u32 bound;
extern pt1Filter_t filter;
extern Kalman kfp,k1,k2,k3,k4,k5,k6;
extern pt1Filter_t filter,f1,f2,f3,f4,f5,f6,f7,f8;
char str1[50];//�ַ�����洢��������ת��
int init_flag=0;//��ʼ����flag
int time_cnt=0;//��ʱ���жϼ�����
//����Ƕ�
extern float Pitch,Roll,Yaw;
//���ٶ�
extern float GYRO[3],ACC[3];
//���ٶȴ�����ԭʼ����
extern float aacx,aacy,aacz;
//������ԭʼ����
extern short gyrox,gyroy,gyroz;	
//������ѹ
extern float volt;
/*
	�������ã���ʱ����ʼ��
	psc��Ԥ��Ƶϵ��
	arr���Զ�װ��ֵ
	Tout= ((arr+1)*(psc+1))/Tclk��
*/
void Timer4_init(int psc,int arr)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//ʹ�ܶ�ʱ��ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=psc;//�Զ�װ��ֵ
  TIM_TimeBaseInitStrue.TIM_Prescaler=arr; //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//��ʼ��
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//������ʱ���ж� �����жϵ��������ʱ�����ж�,����ָ���Ǹ��¼�������ֵ,�ж���ָ������ֵ��ʱ�������ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // ����NVIC�жϷ���2,����2λ��ռ���ȼ���2λ��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;//�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;//��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//ʹ��
  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC
	TIM_Cmd(TIM4,ENABLE);//ʹ�ܶ�ʱ��
}
extern "C"{
void TIM4_IRQHandler(void)//200Hz 0.005s
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //��� TIM4 �����жϷ������
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update ); //��� TIM4 �����жϱ�־
		time_cnt++;
		if(time_cnt%200==0)//������200 ��1s ����LED��˸ �Ա�ʾ��������
		{
			//PEout(0)=!PEout(0);
			time_cnt=1;
		}
		if(init_flag==0)//Ҫ���ж����ȶ�ADC uart�ȳ�ʼ������ֻ��main�� ���޷�������ʼ�����³�����
		{
			EXTIX_Init();
			init_drv_ADC();
			uart_init(bound);
			//ÿ�����ݵĿ������˲����ò�ͬ�Ŀ����������ṹ��
			Kalman_Init(&kfp);
			Kalman_Init(&k1);
			Kalman_Init(&k2);
			Kalman_Init(&k3);
			Kalman_Init(&k4);
			Kalman_Init(&k5);
			Kalman_Init(&k6);
			init_flag++;
		}
		//��̬ƫ����ȡ ���� �����ɺ����
		if(offset_flag==0)
		{
			//PEout(0)=1;//����
			
			if(cnt>1000&&cnt<1200)
			{
				MPU6050_read(ACC,GYRO);
				if(nn==0)//��nn��Ϊ��һ�ζ����ݵ�flag������һ�����ݱ��浽last_gyrox
				{
					last_gyrox=GYRO[0];
					nn++;
				}
				if(fabs(last_gyrox-GYRO[0])>10)//����ֵ�仯����10
					offset_flag=2;
				else last_gyrox=GYRO[0];
				sum1+=GYRO[0];
				sum2+=GYRO[1];
				sum3+=GYRO[2];
				cnt++;
			}
			else if(cnt<=1000)
				cnt++;
			else if(cnt==1200)
				offset_flag++;
		}
		if(offset_flag==1)
		{
			gyro_offset[0]=sum1/200;
			gyro_offset[1]=sum2/200;
			gyro_offset[2]=sum3/200;
		}
		//��ȡ��ѹ
		volt=Get_battery_volt(way);
		//��ȡ�Ƕ�
		if(offset_flag==1){
		if(way==1)                          
		{	
			//��ȡ���ٶȡ����ٶȡ����(������ת)
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
		}			
		else if(way>1&&way<5)
		{
			Yaw=0.0;
			Get_UnfilterAngle(way,&Pitch,&Roll,GYRO,ACC);
			//�������˲�
			if(way==2)		  	
			{
				 Pitch = Kalman_Filter_x(Pitch,GYRO[0]);
				 Roll  = Kalman_Filter_y(Roll,GYRO[1]);
			}
			//�����˲�
			else if(way==3) 
			{  
				Pitch = Complementary_Filter_x(Pitch,GYRO[0]);
				Roll  = Complementary_Filter_y(Roll,GYRO[1]);
			}
			//��ͨ�˲�
			else if(way==4)
			{
				Pitch=pt1FilterApply4(&f7,Pitch,10,0.005);
				Roll=pt1FilterApply4(&f8,Roll,10,0.005);
			}
		}	
		else if(way==5)//����Ԫ����������̬
		{
			if(MPU6050_read(ACC,GYRO))
				{	
					GYRO[0]-=gyro_offset[0];
					GYRO[1]-=gyro_offset[1];
					GYRO[2]-=gyro_offset[2];
				attitude_algorithm1(GYRO,ACC,&Pitch,&Roll,&Yaw);
				}
		}
   }
		//���õ�������ֵ���͵����� �ӳ�1000*0.005s=5s
    if(i<=1400)
			i++;
		else if(i>1400&&i<2000)
		{
//			uart_dataf(ACC[0]);
			uart_dataf(GYRO[0]); 
//			uart_dataf(volt);
//			uart_dataf(Pitch);
//			uart_dataf(Roll);
			i++;
		}

	}
}
}