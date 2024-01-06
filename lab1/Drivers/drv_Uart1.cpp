#include "drv_Uart1.hpp"
#include "sys.h"
void uart_init(u32 bound)
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
//①串口时钟使能，GPIO 时钟使能，复用时钟使能
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|
RCC_APB2Periph_GPIOA, ENABLE); //使能 USART1,GPIOA 时钟
//②串口复位
USART_DeInit(USART1); //复位串口 1
//③GPIO 端口模式设置
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //ISART1_TX PA.9
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化 GPIOA.9
 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //USART1_RX PA.10
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化 GPIOA.10
//④串口参数初始化
USART_InitStructure.USART_BaudRate = bound; //波特率设置
USART_InitStructure.USART_WordLength = USART_WordLength_8b; //字长为 8 位
USART_InitStructure.USART_StopBits = USART_StopBits_1; //一个停止位
USART_InitStructure.USART_Parity = USART_Parity_No; //无奇偶校验位
USART_InitStructure.USART_HardwareFlowControl 
= USART_HardwareFlowControl_None; //无硬件数据流控制
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//收发模式
USART_Init(USART1, &USART_InitStructure); //初始化串口
#if EN_USART1_RX //如果使能了接收
//⑤初始化 NVIC
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ; //抢占优先级 3
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //子优先级 3
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ 通道使能
NVIC_Init(&NVIC_InitStructure); //中断优先级初始化
//⑤开启中断
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启中断
#endif
//⑥使能串口
USART_Cmd(USART1, ENABLE); //使能串口
}