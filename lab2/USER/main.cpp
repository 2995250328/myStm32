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
u32 bound=115200;//波特率变量
u8 t;//作为for循环的变量
u8 len;//记录读取数据的长度
int num;
//温度变量
int Temperature;
//Z轴加速度计  
float Acceleration_Z; 	
//平衡倾角 平衡陀螺仪 转向陀螺仪
float Angle_Balance,Gyro_Balance,Gyro_Turn;
//定时器计数器 每次中断+1 1s后置一 方便调整显示刷新
extern int time_cnt;
extern float gyro_offset[3];
extern float GYRO[3],ACC[3];
float volt=0;//存储电压值的变量
u16 n=0;
u8 OFF[]={"Solving static bias,don't move"};

RCC_ClocksTypeDef RCC_CLK;
int main(void)
{	
	//系统时钟初始化
	SystemInit();
	//延时时钟初始化
  delay_init();	       
	//获取时钟总线
	RCC_GetClocksFreq(&RCC_CLK);//Get chip frequencies	
	//初始化设备驱动
	init_drv_Main();
	//初始化定时器TIM4
	Timer4_init(999,359);//  72m/1000/360=200hz  0.005s
	//初始化屏幕为白色
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	//ADC1的初始化
	init_drv_ADC();
	//按键中断初始化
	EXTIX_Init();
	/*MPU6050初始化*/
	MPU_Init();
	//等待MPU6050初始化完成
	while(mpu_dmp_init());
	//初始化uart1
	uart_init(115200);
	init_drv_Uart2(9600);


	//进入主循环
	while(1)
	{	
		//实现数据回传
		if(USART_RX_STA&0x8000)
		{
			len=USART_RX_STA&0x3f; //得到此次接收到的数据长度
			printf("\r\nthe massage is:\r\n");
			for(t=0;t<len;t++)
			{ 
				USART_SendData(USART1, USART_RX_BUF[t]); //向串口 1 发送数据
				while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
				//等待发送结束
			}
			printf("\r\n"); //插入换行
			USART_RX_STA=0;
		}
		if(offset_flag==0)
			LCD_ShowString(0,128,OFF,RED,BLACK,16,0);
		else if(offset_flag==2)
			LCD_Fill(0,0,LCD_W,LCD_H,RED);
		//显示主要内容
		if((time_cnt%20)==0&&offset_flag==1)
		{
			LCD_Fill(0,128,256,144,WHITE);
			LCD_ShowMAIN();	
		}
	}
}
