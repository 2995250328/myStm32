#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
#include <stdint.h>


#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         		//接收状态标记	
#ifdef __cplusplus
	extern "C" {
#endif
		
/*
 * 函数功能：串口初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void uart_init(u32 bound);
/*
		函数功能：将得到的浮点数据通过串口发送
		入口参数：需要通过串口发送的数据
		无返回值
*/
void uart_dataf(float data);		
/*
		函数功能：将得到的short数据通过串口发送
		入口参数：需要通过串口发送的数据
		无返回值
*/
void uart_datas(short data);
		
#ifdef __cplusplus
	}
#endif
#endif


