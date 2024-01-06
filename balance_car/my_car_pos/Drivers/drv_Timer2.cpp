#include "drv_Timer2.hpp"
#include "drv_ADC.hpp"
#include "drv_Encoder.hpp"
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
int time_cnt=0;//进入中断次数的计数变量
int Volt_flag=0;//电压值小于11.7的flag
extern float Voltage; 
extern float Pitch,Roll,Yaw;
extern int Encoder_Left,Encoder_Right;

extern "C" void TIM2_IRQHandler(void)
{ 		    		  			    
	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{    
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志		
		if(time_cnt<100)//每一百次即0.5s更新一次电压值
		{
			Voltage_temp=Get_battery_volt();
			Voltage_sum+=Voltage_temp;
			time_cnt++;
		}
		else if(time_cnt==100)
		{
			time_cnt=0;
			Voltage=(float)Voltage_sum/100;
			Voltage_sum=0;//置零 防止影响测量
			if(Voltage/100<11.7&&Volt_flag==0)
				Volt_flag=1;
			else
				Volt_flag=0;
			
			PEout(0)=!PEout(0);//计数到100 即0.5s 控制LED闪烁 以表示程序正常
		}
		//读取加速度、角速度、倾角(方向旋转)
		mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
		//读取编码器
		Encoder_Left=-Read_Encoder(4);
		Encoder_Right=Read_Encoder(3);
		#if Debug
			#if Debug_pos
   					Motor_Left=Position_PID(Encoder_Left,Target_Position_L);
				  	Motor_Left=PWM_Limit(Motor_Left,6700,-6700);
//						Motor_Right=Position_PID(Encoder_Right,Target_Position_R);
//    				Motor_Right=PWM_Limit(Motor_Right,6700,-6700);
					  Set_Pwm(Motor_Left,0);
   		#elif Debug_vel
		
			#endif
		#endif
	}				       
}