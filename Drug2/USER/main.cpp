#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "DataScop_DP.hpp"
#include "GUI.hpp"
#include "Control.hpp"
#include "drv_Main.hpp"
#include "drv_Timer2.hpp"
#include "drv_Key.hpp"
#include "drv_OpenMv.hpp"
#include "drv_K210.hpp"
#include "drv_Blooth.hpp"
#include "drv_LCD.hpp"
#include "Show.hpp"

int Debug=0;								  //是否进入调试模式
int Debug_pos=0;							  //是否进入位置调试模式
int Debug_vel=0; 							//是否进入速度调试模式

/*	 
 * WHU-CAR 接口说明
 * PD4  ---> AIN1
 * PC12 ---> AIN2
 * PD11 ---> BIN1
 * PD10 ---> BIN2
 * PE10 ---> STBY
 * PD12 ---> E2A  T4C1
 * PD13 ---> E2B  T4C2
 * PC6  ---> E1A  T3C1
 * PC7  ---> E1B  T3C2
 * PE14 ---> PWMA T1C4
 * PE13 ---> PWMB T1C3
 * PC2  ---> ADC  ADC123IN12 
 PB9 绿灯
 PB8 黄灯
*/                 
//蓝牙遥控相关的变量
u8 Flag_front,Flag_back,Flag_Left,Flag_Right,Flag_velocity=2; 
//电机停止标志位 默认停止 
u8 Flag_Stop=0;     
//电机PWM变量
int Motor_Left,Motor_Right;  
//温度变量
int Temperature;        
//左右编码器的脉冲计数	
int Encoder_Left,Encoder_Right;  
//左右编码器计数统计
int Position_L=0,Position_R=0;
//编码器计数换算为实际距离
int Location_CM_L=0,Location_CM_R=0;
//直行巡线Pwm
int Turn_Pwm=0;
//电池电压采样相关的变量
float Voltage;                
//延时和调参相关变量
int delay_50,delay_flag;
//获取时钟
RCC_ClocksTypeDef RCC_CLK;						
//Z轴加速度计  
float Acceleration_Z;   
//平衡倾角 平衡陀螺仪 转向陀螺仪
float Angle_Balance,Gyro_Balance,Gyro_Turn; 
//车轮速度(mm/s)
float Velocity_Left,Velocity_Right;	
//位置PID控制变量，速度PID控制变量
int Pos_Pwm,Vel_Pwm;
//速度位置pid控制变量
int LV_PWM_L=0,LV_PWM_R=0;
//位置PID系数
float Position_KP_L=100.79,Position_KI_L=0.74,Position_KD_L=135; 	
//速度PID系数
float Velocity_KP_L=127,Velocity_KI_L=11.41,Velocity_KD_L=10; 
//位置PID系数
float Position_KP_R=125.5,Position_KI_R=0.85,Position_KD_R=165; 
//速度PID系数 
float Velocity_KP_R=146,Velocity_KI_R=11.41,Velocity_KD_R=25.6; 
//寻迹PID系数	
float OpenMv_KP=23,OpenMv_KI=0.35,OpenMv_KD=30;
//位置PID设定目标数
int Target_Position_L=0,Target_Position_R=0; 			
//速度PID设定目标数
int Target_Velocity=35;
//从k210传来的数字识别相关变量
int LorR=0;//判断目标数字位于左或者右的变量 1为左 2为右
int Num=0;//识别到的数字
//发送给k210的数据
u8 K210_Uart_Tx_Buffer[8] = {0};
u8 K210_Uart_Rx_Buffer[8]={0};
u8 K210_Uart_Rx_Index=0;
u8 K210_Rx_Data[8];
//蓝牙相关数据
u8 Blooth_Uart_Tx_Buffer[8] = {0};
u8 Blooth_Uart_Rx_Buffer[8]={0};
u8 Blooth_Uart_Rx_Index=0;
u8 Blooth_Rx_Data[8];
//OpenMV相关数据
u8 OpenMv_Uart_Tx_Buffer[8] = {0};
u8 OpenMv_Uart_Rx_Buffer[8]={0};
u8 OpenMv_Uart_Rx_Index=0;
u8 OpenMv_Rx_Data[8];
//发送数据的延时
int Delay_Transmit=0;
//目标房间
int Target_Room=0;
//出发和返回的状态变量 控制状态机
int State_Back=0;
int State_Go=0;
//直行时的Flag
int Straight_Flag=0;
//停止的Flag
int Stop_Flag=0;
//转向的FLag
int Spin_Flag=0;
//装药的Flag
int Load_Flag=0;
//远端小角度转向识别数字
int Far_Flag=0;
//装药后延时的flag
int Delay_Flag=0;
//倒车flag
int Back_Flag=0;
//初始化flag
int Init_Flag=0;
int Little_Flag=0;
int Mission=0;//判断发挥任务
int Car2=0;//2车走
int Finish=1;
extern int delay_count;
int main(void)
{	
	//设置系统中断优先级分组4	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
	//系统时钟初始化
	SystemInit();
	//延时时钟初始化
	delay_init();	
	//打开SWD接口,关闭复用功能
	JTAG_Set(JTAG_SWD_DISABLE);   
	JTAG_Set(SWD_ENABLE);        
	//初始化设备驱动
	init_drv_Main();
	//获取时钟总线
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	
	//k210发送数据包
	K210_Uart_Tx_Buffer[0]=0x2c;
	K210_Uart_Tx_Buffer[1]=0x12;
	K210_Uart_Tx_Buffer[2]=0x01;
	K210_Uart_Tx_Buffer[7]=0x5b;
	//蓝牙发送数据包
	Blooth_Uart_Tx_Buffer[0]=0x2c;
	Blooth_Uart_Tx_Buffer[1]=0x12;
	Blooth_Uart_Tx_Buffer[2]=0x01;
	Blooth_Uart_Tx_Buffer[7]=0x5b;
	//初始化定时器2
	Timer2_init();
	//进入主循环
	while(1)
	{	
		//调试模式的选择
		if (Debug)
		{
			delay_ms(50);
			DataScope(); 
			delay_ms(50);
		}
		else
		{
			LCD_ShowMAIN();
			if(Init_Flag<50)
			{
				K210_Transmit(K210_Uart_Tx_Buffer);
				Init_Flag++;
			}
			else if(Init_Flag==50)
			{
				K210_Uart_Tx_Buffer[2]=0;
				K210_Transmit(K210_Uart_Tx_Buffer);
			}

			if(Delay_Flag==1)
			{
				delay_ms(500);
				Delay_Flag=0;
			}
			if(Load_Flag==1)
			{
				if(Mission==1)
				{
					switch(State_Go)
					{
						case 0:
							Car_Go(144);//走到中段数字前
							State_Go++;
						break;
						case 1:
							if(Stop_Flag==1)
							{
								delay_ms(400);
								if(LorR==1)
									Target_Room='C';
								else if(LorR==2)
									Target_Room='D';
								Car_Go(80);//走到自选停车点
								State_Go++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								PBout(8)=1;//点亮黄灯
								if(Car2==1)//小车2可以走
									State_Go++;
							}
						break;
						case 3:
							PBout(8)=0;
							if(Blooth_Uart_Rx_Buffer[4]==1)
								State_Go++;
						break;
						case 4:
							if(Stop_Flag==1)
							{
								Car_Go(-50);
								State_Go++;
							}
						break;
						case 5:
							if(Stop_Flag==1)
							{
								if(Target_Room=='C')
									Car_Spin(left_90);
								else if(Target_Room=='D')
									Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 6:
							if(Stop_Flag==1)
							{
								Car_Go(35);
								State_Go++;
							}
						break;
						case 7:
							if(Stop_Flag==1)
							{
								Finish=0;
								PEout(0)=0;
							}
						break;
					}
				}
			}
			else if(Load_Flag==2)
			{
				if(Mission==1)
				{
					if(Target_Room=='C')
					{
						switch(State_Back)
						{
							case 0:
						//		Blooth_Uart_Tx_Buffer[2]=0x01;//灭灯准备走
								Car_Go(-35);
								Back_Flag=1;
								State_Back++;
							break;
							case 1:
								if(Stop_Flag==1)
								{
									Back_Flag=0;
									Car_Spin(left_90);
									State_Back++;
								}
							break;
							case 2:
								if(Stop_Flag==1)
								{
							//		Blooth_Uart_Tx_Buffer[4]=0x01;//小车2出发
									Car_Go(142);
									State_Back++;
								}
							break;
							case 3:
								if(Stop_Flag==1)
								{
									Finish=0;
									PBout(9)=1;
									Straight_Flag=0;
									Spin_Flag=0;
								}
							break;
						}
					}
					else if(Target_Room=='D')
					{
						switch(State_Back)
						{
							case 0:
					//			Blooth_Uart_Tx_Buffer[2]=0x01;//灭灯准备走
								Car_Go(-34);
								Back_Flag=1;
								State_Back++;
							break;
							case 1:
								if(Stop_Flag==1)
								{
									Back_Flag=0;
									Car_Spin(right_90);
									State_Back++;
								}
							break;
							case 2:
								if(Stop_Flag==1)
								{
							//		Blooth_Uart_Tx_Buffer[4]=0x01;//小车2出发
									Car_Go(142);
									State_Back++;
								}
							break;
							case 3:
								if(Stop_Flag==1)
								{
									Finish=0;
									Straight_Flag=0;
									Spin_Flag=0;
									PBout(9)=1;
								}
							break;
						}
					}
				}
			}
			if(Mission==2)
				{
					if(Load_Flag==0)
					{
					switch(State_Go)
					{
						case 0:
							if(Car2==1)
							{
								Car_Go(177);
								State_Go++;
							}
						break;
						case 1:
							if(Stop_Flag==1)
							{
								Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 2:
							if(Stop_Flag==1)
							{
								Car_Go(28);
								State_Go++;
							}
						break;
						case 3:
							if(Stop_Flag==1)
							{
								if(Blooth_Uart_Rx_Buffer[5]==1)//小车一走到中段十字路口发送的数据位
								{
									Car_Go(-30);
									State_Go++;
								}
							}
						break;
						case 4:
							if(Stop_Flag==1)
							{
								Car_Spin(left_90);
								State_Go++;
							}
						break;
						case 5:
							if(Stop_Flag==1)
							{
								Car_Go(51);
								State_Go++;
							}
						break;
						case 6:
							if(Stop_Flag==1)
							{
								Little_Flag=1;
								Car_Spin(num_spin1);//转小角度看数字
								K210_Uart_Tx_Buffer[4]=1;//此位作为转头看数字的flag
								State_Go++;
							}
						break;
						case 7:
							if(Stop_Flag==1)
							{
								Little_Flag=1;
								delay_ms(200);
								Car_Spin(num_spin2);
								K210_Uart_Tx_Buffer[4]=0;//此位作为转头看数字的flag
								State_Go++;
							}
						break;
						case 8:
							if(Stop_Flag==1)
							{
								Little_Flag=0;
								Car_Go(33);
								State_Go++;
							}
						break;
						case 9:
							if(Stop_Flag==1)
							{
								if(Far_Flag==1)
									Car_Spin(left_90);
								else if(Far_Flag==2)
									Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 10:
							if(Stop_Flag==1)
							{
								Car_Go(51);
								State_Go++;
							}
						break;
						case 11:
							if(Stop_Flag==1)
							{
								delay_ms(400);//延时一段时间识别数字
								if(Far_Flag==1)
								{
									if(LorR==1)//在左边
										Target_Room='E';
									else if(LorR==2) //在右边
										Target_Room='F';
								}
								else if(Far_Flag==2)
								{
									if(LorR==1)//在左边
										Target_Room='H';
									else if(LorR==2) //在右边
										Target_Room='G';		
								}								
								Car_Go(35);
								State_Go++;
							}
						break;
						case 12:
							if(Stop_Flag==1)
							{
								if(Far_Flag==1)
								{
									if(LorR==1)//在左边
										Target_Room='E';
									else if(LorR==2) //在右边
										Target_Room='F';
								}
								else if(Far_Flag==2)
								{
									if(LorR==1)//在左边
										Target_Room='H';
									else if(LorR==2) //在右边
										Target_Room='G';		
								}			
								if(Target_Room=='E'||Target_Room=='H')
									Car_Spin(left_90);
								else if(Target_Room=='F'||Target_Room=='G')
									Car_Spin(right_90);
								State_Go++;
							}
						break;
						case 13:
							if(Stop_Flag==1)
							{
								Car_Go(33);
								State_Go++;
							}
						break;
						case 14:
							if(Stop_Flag==1)
							{
								Straight_Flag=0;
								Spin_Flag=0;
								PEout(0)=0;
							}
						break;
					}
				}
			}
				else if(Load_Flag==1)
				{
				 if(Target_Room=='E')
				{
				switch(State_Back)
				{
					case 0:
		//				Blooth_Uart_Tx_Buffer[2]=1;//小车2发车
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='F')
			{
				switch(State_Back)
				{
					case 0:
					//	Blooth_Uart_Tx_Buffer[2]=1;//小车2发车
						Car_Go(-35);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='G')
			{
				switch(State_Back)
				{
					case 0:
					//	Blooth_Uart_Tx_Buffer[2]=1;//小车2发车
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(right_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}
			else if(Target_Room=='H')
			{
				switch(State_Back)
				{
					case 0:
			//			Blooth_Uart_Tx_Buffer[2]=1;//小车2发车
						Car_Go(-34);
						Back_Flag=1;
						State_Back++;
					break;
					case 1:
						if(Stop_Flag==1)
						{
							Back_Flag=0;
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 2:
						if(Stop_Flag==1)
						{
							Car_Go(87);
							State_Back++;
						}
					break;
					case 3:
						if(Stop_Flag==1)
						{
							Car_Spin(left_90);
							State_Back++;
						}
					break;
					case 4:
						if(Stop_Flag==1)
						{
							Car_Go(235);
							State_Back++;
						}
					break;
					case 5:
						if(Stop_Flag==1)
						{
							Finish=0;
							PBout(9)=1;
							Straight_Flag=0;
							Spin_Flag=0;
						}
					break;
				}
			}					
			}
		}
	}
}
