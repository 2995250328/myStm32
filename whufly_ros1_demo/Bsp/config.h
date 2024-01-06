#ifndef _CONFIG_H_
#define _CONFIG_H_

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f10x.h"
#include "ring_buffer.h"    
#include "millisecondtimer.h"

typedef enum 
{
	SERIAL1 = 0,
	SERIAL2 = 1,
	SERIAL3 = 2,
	SERIAL_END = 3
}Serial_TypeDef;

#define USE_SERIAL1
#define SERIALn							    					 3
#define STARBOT_SERIAL1					    			 USART1
#define STARBOT_SERIAL1_IRQ				         USART1_IRQn
#define STARBOT_SERIAL1_CLK             	 RCC_APB2Periph_USART1
#define STARBOT_SERIAL1_GPIO_CLK           RCC_APB2Periph_GPIOA
#define STARBOT_SERIAL1_GPIO_PORT          	GPIOA
#define STARBOT_SERIAL1_TX_PIN             GPIO_Pin_9
#define STARBOT_SERIAL1_RX_PIN             GPIO_Pin_10
#define STARBOT_SERIAL1_NVIC				       1

#define STARBOT_SERIAL2					           USART2
#define STARBOT_SERIAL2_IRQ				         USART2_IRQn
#define STARBOT_SERIAL2_CLK             	 RCC_APB1Periph_USART2
#define STARBOT_SERIAL2_GPIO_CLK        	 RCC_APB2Periph_GPIOA
#define STARBOT_SERIAL2_GPIO_PORT      	   GPIOA
#define STARBOT_SERIAL2_TX_PIN             GPIO_Pin_2
#define STARBOT_SERIAL2_RX_PIN             GPIO_Pin_3
#define STARBOT_SERIAL2_NVIC				       2

#define STARBOT_SERIAL3					           USART3
#define STARBOT_SERIAL3_IRQ				         USART3_IRQn
#define STARBOT_SERIAL3_CLK             	 RCC_APB1Periph_USART3
#define STARBOT_SERIAL3_GPIO_CLK        	 RCC_APB2Periph_GPIOB
#define STARBOT_SERIAL3_GPIO_PORT      	   GPIOB
#define STARBOT_SERIAL3_TX_PIN             GPIO_Pin_10
#define STARBOT_SERIAL3_RX_PIN             GPIO_Pin_11
#define STARBOT_SERIAL3_NVIC				       3
#define RXBUF_SIZE        				       1024

#endif 

