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
char operation='+';//操作符变量
int operative_index=1;//操作符变换的索引
int number1[3]={0};//用来存储第一个数字
int number2[3]={0};//用来存储第二个数字
int indication=0;//作为操作位切换的指示
float result=0;
int show1=1,show2=1,show3=1;//各张图片的显示标志
int num=0;
u8 str[]={"indication:"};
uint16_t cnt=0;//测试变量

u16 n=0;

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
	//LED初始化
	LED_Init();
	//初始化定时器TIM4
	Timer_init();
	//初始化屏幕为白色
	LCD_Fill(0,0,LCD_W,LCD_H,WHITE);
	//初始化蜂鸣器
//	BEEP_Init();
	//按键中断初始化
	EXTIX_Init();
	//init_drv_Key();
	u32 bound=115200;//波特率变量
	uart_init(115200);
	//进入主循环
	while(1)
	{
		if(USART_RX_STA&0x8000)
 { len=USART_RX_STA&0x3f; //得到此次接收到的数据长度
printf("\r\n message:\r\n\r\n");
for(t=0;t<len;t++)
{ USART_SendData(USART1, USART_RX_BUF[t]); //向串口 1 发送数据
 while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);
//等待发送结束
}
printf("\r\n\r\n"); //插入换行
USART_RX_STA=0;}
	}
}
