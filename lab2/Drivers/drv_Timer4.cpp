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

float last_gyrox=0;//记录上次的陀螺仪数据
int nn=0;
int offset_flag=0;
float sum1=0,sum2=0,sum3=0;//求和
int cnt=0;
extern float gyro_offset[3];
int i=0;
extern int flag;
extern int way;//决定算法的变量
extern u32 bound;
extern pt1Filter_t filter;
extern Kalman kfp,k1,k2,k3,k4,k5,k6;
extern pt1Filter_t filter,f1,f2,f3,f4,f5,f6,f7,f8;
char str1[50];//字符数组存储浮点数的转换
int init_flag=0;//初始化的flag
int time_cnt=0;//定时器中断计数器
//三轴角度
extern float Pitch,Roll,Yaw;
//角速度
extern float GYRO[3],ACC[3];
//加速度传感器原始数据
extern float aacx,aacy,aacz;
//陀螺仪原始数据
extern short gyrox,gyroy,gyroz;	
//测量电压
extern float volt;
/*
	函数作用：定时器初始化
	psc：预分频系数
	arr：自动装载值
	Tout= ((arr+1)*(psc+1))/Tclk；
*/
void Timer4_init(int psc,int arr)
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//使能定时器时钟
	
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStrue;
	TIM_TimeBaseInitStrue.TIM_Period=psc;//自动装载值
  TIM_TimeBaseInitStrue.TIM_Prescaler=arr; //预分频系数
  TIM_TimeBaseInitStrue.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStrue.TIM_ClockDivision=TIM_CKD_DIV1;
  TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStrue);//初始化
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);//开启定时器中断 更新中断当计数溢出时进入中断,更新指的是更新计数器的值,中断是指更新数值的时候会进入中断
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 设置NVIC中断分组2,其中2位抢占优先级，2位响应优先级
  NVIC_InitStructure.NVIC_IRQChannel                   = TIM4_IRQn;//中断通道
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;//响应优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;//使能
  NVIC_Init(&NVIC_InitStructure);//初始化NVIC
	TIM_Cmd(TIM4,ENABLE);//使能定时器
}
extern "C"{
void TIM4_IRQHandler(void)//200Hz 0.005s
{
	if (TIM_GetITStatus(TIM4, TIM_IT_Update) == SET) //检查 TIM4 更新中断发生与否
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update ); //清除 TIM4 更新中断标志
		time_cnt++;
		if(time_cnt%200==0)//计数到200 即1s 控制LED闪烁 以表示程序正常
		{
			//PEout(0)=!PEout(0);
			time_cnt=1;
		}
		if(init_flag==0)//要在中断中先对ADC uart等初始化，若只在main中 则无法正常初始化导致程序卡死
		{
			EXTIX_Init();
			init_drv_ADC();
			uart_init(bound);
			//每个数据的卡尔曼滤波采用不同的卡尔曼参数结构体
			Kalman_Init(&kfp);
			Kalman_Init(&k1);
			Kalman_Init(&k2);
			Kalman_Init(&k3);
			Kalman_Init(&k4);
			Kalman_Init(&k5);
			Kalman_Init(&k6);
			init_flag++;
		}
		//静态偏置求取 灯灭 求解完成后灯闪
		if(offset_flag==0)
		{
			//PEout(0)=1;//灯灭
			
			if(cnt>1000&&cnt<1200)
			{
				MPU6050_read(ACC,GYRO);
				if(nn==0)//用nn作为第一次读数据的flag，将第一次数据保存到last_gyrox
				{
					last_gyrox=GYRO[0];
					nn++;
				}
				if(fabs(last_gyrox-GYRO[0])>10)//两次值变化大于10
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
		//获取电压
		volt=Get_battery_volt(way);
		//获取角度
		if(offset_flag==1){
		if(way==1)                          
		{	
			//读取加速度、角速度、倾角(方向旋转)
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
		}			
		else if(way>1&&way<5)
		{
			Yaw=0.0;
			Get_UnfilterAngle(way,&Pitch,&Roll,GYRO,ACC);
			//卡尔曼滤波
			if(way==2)		  	
			{
				 Pitch = Kalman_Filter_x(Pitch,GYRO[0]);
				 Roll  = Kalman_Filter_y(Roll,GYRO[1]);
			}
			//互补滤波
			else if(way==3) 
			{  
				Pitch = Complementary_Filter_x(Pitch,GYRO[0]);
				Roll  = Complementary_Filter_y(Roll,GYRO[1]);
			}
			//低通滤波
			else if(way==4)
			{
				Pitch=pt1FilterApply4(&f7,Pitch,10,0.005);
				Roll=pt1FilterApply4(&f8,Roll,10,0.005);
			}
		}	
		else if(way==5)//用四元数法解算姿态
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
		//将得到的数据值发送到串口 延迟1000*0.005s=5s
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