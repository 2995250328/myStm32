#include "sys.h"
#include "usart.h"	  
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
__ASM (".global __use_no_semihosting");             
//标准库需要的支持函数                 
//struct __FILE 
//{ 
//	int handle; 
//}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch,FILE *f)
{
	//循环发送,直到发送完毕 
	while((USART1->SR&0X40)==0){}  
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*使用microLib的方法*/
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
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
  
void uart_init(u32 bound)
{
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟

	//USART1_TX   GPIOA.9
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9

	//USART1_RX	  GPIOA.10初始化
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

	//Usart1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器

	//USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
	
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}
//extern u8 i;
//extern int uart4_state;
//extern u8 K210_Uart_Rx_Buffer[8];
//extern u8 K210_Uart_Rx_Index;
//extern u8 K210_Rx_Data[8];
//void USART1_IRQHandler(void)                	//串口1中断服务程序
//{
//	PEout(0)=!PEout(0);
//#if SYSTEM_SUPPORT_OS 		//è?1?SYSTEM_SUPPORT_OS?a??￡??òDèòa?§3?OS.
//	OSIntEnter();    
//#endif
//		u8 com_data; 							
//		static uint8_t RxCounter1=0;
//		static uint8_t RxBuffer1[9]={0};		
//		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //?óê??D??(?óê?μ?μ?êy?Y±?D?ê?0x0d 0x0a?á?2)
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
//    if (!K210_Uart_Rx_Index) // ??óDêy?Y í?3?
//		{    return;
//		}
//    // °′DèDT?? ￡¨7￡oá???short￡? 5￡oò???short￡?òà′?ààí?￡
//    if (K210_Uart_Rx_Index < 7) // ?óê?3¤?èê?·?′?μ?
//        return;

//    // ?ì2é??ê?ê?·??yè·
//    if ((K210_Uart_Rx_Buffer[0] != 0x2C) && (K210_Uart_Rx_Buffer[1] != 0x12) && // ??í·(á???×??ú)
//        (K210_Uart_Rx_Buffer[K210_Uart_Rx_Index - 1] != 0x5B))                  // ???2
//        goto Send_Error;

//    // ?ì2é?üá?êy?Y·??§ê?·?o?àí
//    if ((K210_Uart_Rx_Index - 3) % 2)
//        goto Send_Error;

//    // ?üá??yè·
//    K210_Rx_Data_Analysis_State = 1; // ?a??×′ì???1
//    for (i = 0; i < ((K210_Uart_Rx_Index - 3) >> 1); ++i)
//    {
//        K210_Rx_Data[i] = K210_Uart_Rx_Buffer[2 + (i << 1) + 1] << 8; // ??8??
//        K210_Rx_Data[i] |= K210_Uart_Rx_Buffer[2 + (i << 1) + 0];     // μí8??
//    }

//    goto Jump; // ?y3￡á÷3ì?íì?1y
//Send_Error:    // ?a??′í?ó
//Jump:
//    // ??á??óê??o3? ?a??ò?′??óê?×?±?
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

