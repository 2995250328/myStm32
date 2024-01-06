#include "drv_Uart1.hpp"
#include "sys.h"
void uart_init(u32 bound)
{
GPIO_InitTypeDef GPIO_InitStructure;
USART_InitTypeDef USART_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
//�ٴ���ʱ��ʹ�ܣ�GPIO ʱ��ʹ�ܣ�����ʱ��ʹ��
RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|
RCC_APB2Periph_GPIOA, ENABLE); //ʹ�� USART1,GPIOA ʱ��
//�ڴ��ڸ�λ
USART_DeInit(USART1); //��λ���� 1
//��GPIO �˿�ģʽ����
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //ISART1_TX PA.9
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ�� GPIOA.9
 
GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //USART1_RX PA.10
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //��������
GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ�� GPIOA.10
//�ܴ��ڲ�����ʼ��
USART_InitStructure.USART_BaudRate = bound; //����������
USART_InitStructure.USART_WordLength = USART_WordLength_8b; //�ֳ�Ϊ 8 λ
USART_InitStructure.USART_StopBits = USART_StopBits_1; //һ��ֹͣλ
USART_InitStructure.USART_Parity = USART_Parity_No; //����żУ��λ
USART_InitStructure.USART_HardwareFlowControl 
= USART_HardwareFlowControl_None; //��Ӳ������������
USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;//�շ�ģʽ
USART_Init(USART1, &USART_InitStructure); //��ʼ������
#if EN_USART1_RX //���ʹ���˽���
//�ݳ�ʼ�� NVIC
NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ; //��ռ���ȼ� 3
NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ� 3
NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ ͨ��ʹ��
NVIC_Init(&NVIC_InitStructure); //�ж����ȼ���ʼ��
//�ݿ����ж�
USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //�����ж�
#endif
//��ʹ�ܴ���
USART_Cmd(USART1, ENABLE); //ʹ�ܴ���
}