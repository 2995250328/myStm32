#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
#include <stdint.h>


#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
#ifdef __cplusplus
	extern "C" {
#endif
		
/*
 * �������ܣ����ڳ�ʼ��
 * ��ڲ�����������
 * ����  ֵ����
 */
void uart_init(u32 bound);
/*
		�������ܣ����õ��ĸ�������ͨ�����ڷ���
		��ڲ�������Ҫͨ�����ڷ��͵�����
		�޷���ֵ
*/
void uart_dataf(float data);		
/*
		�������ܣ����õ���short����ͨ�����ڷ���
		��ڲ�������Ҫͨ�����ڷ��͵�����
		�޷���ֵ
*/
void uart_datas(short data);
		
#ifdef __cplusplus
	}
#endif
#endif


