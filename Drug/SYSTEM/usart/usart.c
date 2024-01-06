#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
__ASM (".global __use_no_semihosting");             
//��׼����Ҫ��֧�ֺ���                 
//struct __FILE 
//{ 
//	int handle; 
//}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch,FILE *f)
{
	//ѭ������,ֱ��������� 
	while((USART1->SR&0X40)==0){}  
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
  
void uart_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9

	//USART1_RX	  GPIOA.10��ʼ��
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

	//Usart1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	//USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}
//extern u8 i;
//extern int uart4_state;
//extern u8 K210_Uart_Rx_Buffer[8];
//extern u8 K210_Uart_Rx_Index;
//extern u8 K210_Rx_Data[8];
//void USART1_IRQHandler(void)                	//����1�жϷ������
//{
//	PEout(0)=!PEout(0);
//#if SYSTEM_SUPPORT_OS 		//��?1?SYSTEM_SUPPORT_OS?a??��??��D����a?��3?OS.
//	OSIntEnter();    
//#endif
//		u8 com_data; 							
//		static uint8_t RxCounter1=0;
//		static uint8_t RxBuffer1[9]={0};		
//		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //?����??D??(?����?��?��?��y?Y��?D?��?0x0d 0x0a?��?2)
//		{
//			com_data = USART_ReceiveData(USART1);
//			if(uart4_state==0&&com_data==0x2C)
//			{
//				uart4_state=1;
//				K210_Uart_Rx_Buffer[RxCounter1++]=com_data;//0
//			}
//			else if(uart4_state==1&&com_data==0x12)
//			{
//				uart4_state=2;
//				K210_Uart_Rx_Buffer[RxCounter1++]=com_data; //1
//			}				
//			else if(uart4_state==2)
//			{
//					K210_Uart_Rx_Buffer[RxCounter1++]=com_data;
//					if(RxCounter1>=9 || com_data == 0x5B)       //RxBuffer1????,??????
//					{
//				  uart4_state=3;
//					}
//			}
//			else if(uart4_state==3)						
//			{
//				if(K210_Uart_Rx_Buffer[RxCounter1-1] == 0x5B)
//				{
//							
//					RxCounter1 = 0;
//					uart4_state = 0;
//				//	K210_Process(RxBuffer1);
//				}
//				else   
//				{
//					uart4_state = 0;
//					RxCounter1=0;
//				}
//			} 
//			else  
//			{
//				uart4_state = 0;
//				RxCounter1=0;
//			}
//				
//		}		
//		i=com_data;
//		  if (K210_Uart_Rx_Index < 50) // ????????
//    {
//        K210_Uart_Rx_Index=RxCounter1;                                            // ???1
//    }
//#if SYSTEM_SUPPORT_OS 	
//	OSIntExit();  											 
//#endif
//} 
//extern int K210_Rx_Data_Analysis_State;
//void K210_Check_Data_Task(void)
//{
//    uint16_t i;
//    if (!K210_Uart_Rx_Index) // ??��D��y?Y ��?3?
//		{    return;
//		}
//    // ���D��DT?? �ꡧ7��o��???short��? 5��o��???short��?������?������?��
//    if (K210_Uart_Rx_Index < 7) // ?����?3��?����?��?��?��?
//        return;

//    // ?��2��??��?��?��??y����
//    if ((K210_Uart_Rx_Buffer[0] != 0x2C) && (K210_Uart_Rx_Buffer[1] != 0x12) && // ??����(��???��??��)
//        (K210_Uart_Rx_Buffer[K210_Uart_Rx_Index - 1] != 0x5B))                  // ???2
//        goto Send_Error;

//    // ?��2��?����?��y?Y��??�쨺?��?o?����
//    if ((K210_Uart_Rx_Index - 3) % 2)
//        goto Send_Error;

//    // ?����??y����
//    K210_Rx_Data_Analysis_State = 1; // ?a??���䨬???1
//    for (i = 0; i < ((K210_Uart_Rx_Index - 3) >> 1); ++i)
//    {
//        K210_Rx_Data[i] = K210_Uart_Rx_Buffer[2 + (i << 1) + 1] << 8; // ??8??
//        K210_Rx_Data[i] |= K210_Uart_Rx_Buffer[2 + (i << 1) + 0];     // �̨�8??
//    }

//    goto Jump; // ?y3�ꨢ��3��?����?1y
//Send_Error:    // ?a??�䨪?��
//Jump:
//    // ??��??����??o3? ?a??��?��??����?��?��?
//    K210_Uart_Rx_Index = 0;
//		for(int i=0;i<8;i++)
//			K210_Uart_Rx_Buffer[i]=0;
//		
//		if(K210_Rx_Data_Analysis_State)
//		{
//			K210_Rx_Data_Analysis_State=0;
//		}
//}
#endif	

