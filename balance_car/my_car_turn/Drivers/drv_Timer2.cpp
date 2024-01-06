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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//使能定时器时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=999;//自动装载值
  TIM_TimeBaseInitStrue.TIM_Prescaler=359; //预分频系数
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStrue);//初始化
	TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//开启定时器中断 更新中断当计数溢出时进入中断,更新指的是更新计数器的值,中断是指更新数值的时候会进入中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 设置NVIC中断分组,其中0位抢占优先级，0位响应优先级
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM2_IRQn;//中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//使能
  NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	TIM_Cmd(TIM2,ENABLE);//使能定时器
}

int Voltage_temp;//存储每一个定时器中断获得的电压值
int Voltage_sum=0;//存储100个定时器中断获得的总电压值，从而取平均
int Volt_flag=0;//电压值小于11.7的flag
int time_cnt=0;//进入中断次数的计数变量
int offset_cnt=0;//进行静态偏置读取的计数变量
int offset_flag=0;//静态偏置是否完成的flag
int first_flag=0;//第一次读取的flag
int init_flag=0;//对dmp前段时间读取的值不进行操作
int Voltage_Count=0;
int V_flag=0;//电压测量flag
float sumx=0,sumy=0,sumz=0;//求和
float last_gyrox=0,last_gyroy=0,last_gyroz=0;//记录上次的陀螺仪数据
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
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志	
		float gyro_bias=0;//三轴角度变化值的几何平均值
		if(offset_flag==0)
		{
			if(offset_cnt<600)//600个周期即3s 等待初始化完成以及读数稳定
				offset_cnt++;
			else if(offset_cnt>=600&&offset_cnt<1000)//读取400次
			{
				MPU6050_read(ACC,GYRO);
				if(first_flag==0)//用nn作为第一次读数据的flag，将第一次数据保存到last_gyrox
				{
					last_gyrox=GYRO[0];
					last_gyroy=GYRO[1];
					last_gyroz=GYRO[2];
					first_flag++;
				}
				//√x_bias^2+y_bias^2+z_bias^2
				gyro_bias=sqrt((last_gyrox-GYRO[0])*(last_gyrox-GYRO[0])+(last_gyroy-GYRO[1])*(last_gyroy-GYRO[1])+(last_gyroz-GYRO[2])*(last_gyroz-GYRO[2]));
				if(gyro_bias>7.5)//两次值变化大于7.5
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
			else if(offset_flag!=2&&offset_cnt==1000)//之前没有移动 且读取到了400个数据
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
				if(init_flag<50)//先读取50*0.005s等待稳定
				{
					//读取加速度、角速度、倾角(方向旋转) 存储在Pitch，Yaw，Angle_Balance=Pitch，Gyro_Turn=Gyro_Z，Acceleration_Z=Accel_Z，Gyro_Balance=Gyro_X
					Get_Angle(1);
					init_flag++;
				}
				else 
				{
					//读取加速度、角速度、倾角(方向旋转) 存储在Pitch，Yaw，Angle_Balance=Pitch，Gyro_Turn=Gyro_Z，Acceleration_Z=Accel_Z，Gyro_Balance=Gyro_X
					Get_Angle(2);
					if(Yaw<0)//将yaw转化为360度制
						Yaw=Yaw+360;
					//读取编码器
					Encoder_Left=Read_Encoder(4);
					Encoder_Right=Read_Encoder(3);
					if(Uart1_Receive==97)//只有当蓝牙发送a时才会改变目标值
						Turn_Target=Turn_Target_Tem;
					//计算速度
					Get_Velocity_Form_Encoder(Encoder_Left,Encoder_Right);
					//三环控制
					Balance_Pwm=Balance(Angle_Balance,Gyro_Balance);    //平衡PID控制 Gyro_Balance平衡角速度极性：前倾为正，后倾为负
					Velocity_Pwm=Velocity(Encoder_Left,Encoder_Right);  //速度环PID控制	
					if(Uart1_Receive==97)//只有当蓝牙发送a时才会改变目标值
					{
						Turn_Target=Turn_Target_Tem;
						Turn_Pwm=Turn(Yaw,Gyro_Turn);												//转向环PID控制
						Turn_Pwm=PWM_Limit(Turn_Pwm,2000,-2000);
					}
					//计算最终的pwm
					Motor_Left=Balance_Pwm+Velocity_Pwm+Turn_Pwm;       //计算左轮电机最终PWM
					Motor_Right=Balance_Pwm+Velocity_Pwm-Turn_Pwm;      //计算右轮电机最终PWM
					Motor_Left=PWM_Limit(Motor_Left,6900,-6900);
					Motor_Right=PWM_Limit(Motor_Right,6900,-6900);			//PWM限幅
					if(Turn_Off(Angle_Balance,Volt_flag)==0)     					//如果正常 赋值
						Set_Pwm(Motor_Left,Motor_Right);    					
				}
			}
		}
	}				       
}