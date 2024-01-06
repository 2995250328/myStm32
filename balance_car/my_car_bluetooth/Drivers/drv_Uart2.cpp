#include "Basic.hpp"
#include "drv_Uart2.hpp"
#include "Commulink.hpp"
u8 Uart2_Receive;
/*
 * 函数功能：串口2初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart2(u32 bound)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//使能USART2，GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//重映射使能
	GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
	//USART2_TX   GPIOD.5
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //USART2_RX	  GPIOD.6初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	//初始化串口2
  USART_Init(USART2, &USART_InitStructure); 
	//开启串口接受中断
  //USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);
	//使能串口2 
  USART_Cmd(USART2, ENABLE);                    
}
extern u8 Flag_front,Flag_back,Flag_Left,Flag_Right;
extern "C" void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) 
	{
		//蓝牙接收相关变量
	  static int uart_receive=0;
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
  	uart_receive=USART_ReceiveData(USART2); 
		Uart2_Receive=uart_receive;	
		if(uart_receive==0x5A)//Z	    
			Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;	//刹车
		else if(uart_receive==0x41)//A	
			Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;	//前
		else if(uart_receive==0x45)//E	
			Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;	//后
		else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)//B  C  D	
			Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //右
		else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)//F  G  H	    
			Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //左
		else 
			Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;	//刹车
	}
}