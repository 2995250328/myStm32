#include "Basic.hpp"
#include "drv_Uart1.hpp"
#include "Commulink.hpp"
#include "drv_MPU6050.hpp"
#include "Control.hpp"	

u8 Uart1_Receive;
/*
 * �������ܣ�����1��ʼ��
 * ��ڲ�����������
 * ����  ֵ����
 */
void init_drv_Uart1(u32 bound)
{
	//GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	//ʹ��USART1��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	//��ʼ������1
  USART_Init(USART1, &USART_InitStructure); 
	//�������ڽ����ж�
  //USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	//ʹ�ܴ���1 
  USART_Cmd(USART1, ENABLE);                    
}

/*
 * �������ܣ�����1�����ж�
 * ��ڲ�������
 * ����  ֵ����
 */
//extern "C" void USART1_IRQHandler(void)
//{	
//	//���յ�����
//	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
//	{	  
//		//����������ر���
//	  static int uart_receive=0;
//		static u8 Flag_PID,i,j,Receive[50];
//		static float Data;
//  	uart_receive=USART_ReceiveData(USART1); 
//		Uart1_Receive=uart_receive;
//		//���ٵ���Ĭ��ֵ��
//		if(uart_receive==0x59)  
//			Flag_velocity=2;  
//		//���ٵ�
//		if(uart_receive==0x58)  
//			Flag_velocity=1;  
//		//Ĭ��ʹ��
//	  if(uart_receive>10)  
//    {			
//			if(uart_receive==0x5A)	    
//				Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;	//ɲ��
//			else if(uart_receive==0x41)	
//				Flag_front=1,Flag_back=0,Flag_Left=0,Flag_Right=0;	//ǰ
//			else if(uart_receive==0x45)	
//				Flag_front=0,Flag_back=1,Flag_Left=0,Flag_Right=0;	//��
//			else if(uart_receive==0x42||uart_receive==0x43||uart_receive==0x44)	
//				Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=1;  //��
//			else if(uart_receive==0x46||uart_receive==0x47||uart_receive==0x48)	    
//				Flag_front=0,Flag_back=0,Flag_Left=1,Flag_Right=0;  //��
//			else 
//				Flag_front=0,Flag_back=0,Flag_Left=0,Flag_Right=0;	//ɲ��
//  	}
//	}  											 
//} 
