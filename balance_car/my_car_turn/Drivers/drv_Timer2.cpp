#include <math.h>
#include "drv_Timer2.hpp"
#include "drv_ADC.hpp"
#include "drv_Encoder.hpp"
#include "drv_MPU6050.hpp"
#include "filter.h"
#include "usart.h"
#include "Control.hpp"
#include "inv_mpu.h"
#include "stm32f10x.h"
#include "sys.h"
void Timer2_init()// 72M/1000/360=200hz  0.005s
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//ʹ�ܶ�ʱ��ʱ��
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=999;//�Զ�װ��ֵ
  TIM_TimeBaseInitStrue.TIM_Prescaler=359; //Ԥ��Ƶϵ��
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//���ϼ���
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrue);//��ʼ��
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //��� TIM2 �����жϱ�־		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//������ʱ���ж� �����жϵ��������ʱ�����ж�,����ָ���Ǹ��¼�������ֵ,�ж���ָ������ֵ��ʱ�������ж�
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // ����NVIC�жϷ���,����0λ��ռ���ȼ���0λ��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;//�ж�ͨ��
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//��ռ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;//��Ӧ���ȼ�
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//ʹ��
  NVIC_Init(&NVIC_InitStructure);//��ʼ��NVIC
	TIM_Cmd(TIM2,ENABLE);//ʹ�ܶ�ʱ��
}

int Voltage_temp;//�洢ÿһ����ʱ���жϻ�õĵ�ѹֵ
int Voltage_sum=0;//�洢100����ʱ���жϻ�õ��ܵ�ѹֵ���Ӷ�ȡƽ��
int Volt_flag=0;//��ѹֵС��11.7��flag
int time_cnt=0;//�����жϴ����ļ�������
int offset_cnt=0;//���о�̬ƫ�ö�ȡ�ļ�������
int offset_flag=0;//��̬ƫ���Ƿ���ɵ�flag
int first_flag=0;//��һ�ζ�ȡ��flag
int init_flag=0;//��dmpǰ��ʱ���ȡ��ֵ�����в���
int Voltage_Count=0;
int V_flag=0;//��ѹ����flag
float sumx=0,sumy=0,sumz=0;//���
float last_gyrox=0,last_gyroy=0,last_gyroz=0;//��¼�ϴε�����������
float Position_L=0,Position_R=0;
float Position_Moto_L=0,Position_Moto_R=0;
float limit_l,limit_r;
extern int key_flag;
extern float gyro_offset[3];
extern float GYRO[3],ACC[3];
extern float Voltage; 
extern float Pitch,Roll,Yaw;
extern int Encoder_Left,Encoder_Right;
extern float Turn_Target_Tem,Trun_Target;
extern u8 Uart1_Receive;
extern "C" void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{    
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //��� TIM2 �����жϱ�־	
		float gyro_bias=0;//����Ƕȱ仯ֵ�ļ���ƽ��ֵ
		if(offset_flag==0)
		{
			if(offset_cnt<600)//600�����ڼ�3s �ȴ���ʼ������Լ������ȶ�
				offset_cnt++;
			else if(offset_cnt>=600&&offset_cnt<1000)//��ȡ400��
			{
				MPU6050_read(ACC,GYRO);
				if(first_flag==0)//��nn��Ϊ��һ�ζ����ݵ�flag������һ�����ݱ��浽last_gyrox
				{
					last_gyrox=GYRO[0];
					last_gyroy=GYRO[1];
					last_gyroz=GYRO[2];
					first_flag++;
				}
				//��x_bias^2+y_bias^2+z_bias^2
				gyro_bias=sqrt((last_gyrox-GYRO[0])*(last_gyrox-GYRO[0])+(last_gyroy-GYRO[1])*(last_gyroy-GYRO[1])+(last_gyroz-GYRO[2])*(last_gyroz-GYRO[2]));
				if(gyro_bias>7.5)//����ֵ�仯����7.5
					offset_flag=2;
				else 
				{
					last_gyrox=GYRO[0];
					last_gyroy=GYRO[1];
					last_gyroz=GYRO[2];
				}
				sumx+=GYRO[0];
				sumy+=GYRO[1];
				sumz+=GYRO[2];
				offset_cnt++;
			}
			else if(offset_flag!=2&&offset_cnt==1000)//֮ǰû���ƶ� �Ҷ�ȡ����400������
			{
				offset_flag=1;
				gyro_offset[0]=sumx/400;
				gyro_offset[1]=sumy/400;
				gyro_offset[2]=sumz/400;
			}
		}
		if(time_cnt<200)
			time_cnt++;
		else if(time_cnt==200)
		{
			PEout(0)=!PEout(0);
			time_cnt=0;
		}
		V_flag=!V_flag;
		if(V_flag==1)
		{
			Voltage_temp=Get_battery_volt();
			Voltage_Count++;
			Voltage_sum+=Voltage_temp;
			if(Voltage_Count==100)
			{
				Voltage=Voltage_sum/100,Voltage_sum=0,Voltage_Count=0;
				if(Voltage/100<=11.1&&Volt_flag==0)
					Volt_flag=1;
			}
		}
		else if(V_flag==0)
		{
			if(offset_flag==1&&Volt_flag==0)
			{
				if(init_flag<50)//�ȶ�ȡ50*0.005s�ȴ��ȶ�
				{
					//��ȡ���ٶȡ����ٶȡ����(������ת) �洢��Pitch��Yaw��Angle_Balance=Pitch��Gyro_Turn=Gyro_Z��Acceleration_Z=Accel_Z��Gyro_Balance=Gyro_X
					Get_Angle(1);
					init_flag++;
				}
				else 
				{
					//��ȡ���ٶȡ����ٶȡ����(������ת) �洢��Pitch��Yaw��Angle_Balance=Pitch��Gyro_Turn=Gyro_Z��Acceleration_Z=Accel_Z��Gyro_Balance=Gyro_X
					Get_Angle(2);
					if(Yaw<0)//��yawת��Ϊ360����
						Yaw=Yaw+360;
					//��ȡ������
					Encoder_Left=Read_Encoder(4);
					Encoder_Right=Read_Encoder(3);
					if(Uart1_Receive==97)//ֻ�е���������aʱ�Ż�ı�Ŀ��ֵ
						Turn_Target=Turn_Target_Tem;
					//�����ٶ�
					Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
					//��������
					Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //ƽ��PID���� Gyro_Balanceƽ����ٶȼ��ԣ�ǰ��Ϊ��������Ϊ��
					Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //�ٶȻ�PID����	
					if(Uart1_Receive==97)//ֻ�е���������aʱ�Ż�ı�Ŀ��ֵ
					{
						Turn_Target=Turn_Target_Tem;
						Turn_Pwm=Turn(Yaw,Gyro_Turn);												//ת��PID����
						Turn_Pwm=PWM_Limit(Turn_Pwm,2000,-2000);
					}
					//�������յ�pwm
					Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //�������ֵ������PWM
					Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //�������ֵ������PWM
					Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
					Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM�޷�
					if(Turn_Off(Angle_Balance,Volt_flag)==0)     					//������� ��ֵ
						Set_Pwm(Motor_Left,Motor_Right);    					
				}
			}
		}
	}				       
}