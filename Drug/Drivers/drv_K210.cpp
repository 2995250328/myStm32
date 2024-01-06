#include "Basic.hpp"
#include "drv_K210.hpp"
#include "Commulink.hpp"

/*
 * 函数功能：串口2初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart4(u32 bound)	

{
		//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

//使能USART4，GPIOC时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	
	//USART4_TX   GPIOC.10
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //USART4_RX	  GPIOC.11
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

	//USART4 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
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

  //初始化串口4
  USART_Init(UART4, &USART_InitStructure); 
	//开启串口接受中断
  USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);
	//使能串口4
  USART_Cmd(UART4, ENABLE);                 //使能串口1 
}
u8 i;
int uart4_state;
extern u8 K210_Uart_Rx_Buffer[8];
extern u8 K210_Uart_Rx_Index;
extern u8 K210_Rx_Data[8];
extern "C"{
void UART4_IRQHandler(void)                	//串口1中断服务程序
{
//	PEout(0)=!PEout(0);
#if SYSTEM_SUPPORT_OS 		//è?1?SYSTEM_SUPPORT_OS?a??￡??òDèòa?§3?OS.
	OSIntEnter();    
#endif
		u8 com_data; 							
		static uint8_t RxCounter1=0;
		static uint8_t RxBuffer1[9]={0};		
		if(USART_GetITStatus(UART4, USART_IT_RXNE) != RESET)  //?óê??D??(?óê?μ?μ?êy?Y±?D?ê?0x0d 0x0a?á?2)
		{
			com_data = USART_ReceiveData(UART4);
			if(uart4_state==0&&com_data==0x2C)
			{
				uart4_state=1;
				K210_Uart_Rx_Buffer[RxCounter1++]=com_data;//0
			}
			else if(uart4_state==1&&com_data==0x12)
			{
				uart4_state=2;
				K210_Uart_Rx_Buffer[RxCounter1++]=com_data; //1
			}				
			else if(uart4_state==2)
			{
					K210_Uart_Rx_Buffer[RxCounter1++]=com_data;
					if(RxCounter1>=9 || com_data == 0x5B)       //RxBuffer1????,??????
					{
				  uart4_state=3;
					}
			}
			else if(uart4_state==3)						
			{
				if(K210_Uart_Rx_Buffer[RxCounter1-1] == 0x5B)
				{
							
					RxCounter1 = 0;
					uart4_state = 0;
				//	K210_Process(RxBuffer1);
				}
				else   
				{
					uart4_state = 0;
					RxCounter1=0;
				}
			} 
			else  
			{
				uart4_state = 0;
				RxCounter1=0;
			}
				
		}		
		i=com_data;
		  if (K210_Uart_Rx_Index < 50) // ????????
    {
        K210_Uart_Rx_Index=RxCounter1;                                            // ???1
    }
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
} 
}
extern int LorR;
extern int Num;
extern u8 K210_Uart_Tx_Buffer[8];
extern int Target_Room;
extern int Far_Flag;
extern int Load_Flag;
void K210_Check_Data_Task(void)
{
    uint16_t i;
    if (!K210_Uart_Rx_Index) // ??óDêy?Y í?3?
		{    return;
		}
    // °′DèDT?? ￡¨7￡oá???short￡? 5￡oò???short￡?òà′?ààí?￡
    if (K210_Uart_Rx_Index < 7) // ?óê?3¤?èê?·?′?μ?
        return;

    // ?ì2é??ê?ê?·??yè·
    if ((K210_Uart_Rx_Buffer[0] != 0x2C) && (K210_Uart_Rx_Buffer[1] != 0x12) && // ??í·(á???×??ú)
        (K210_Uart_Rx_Buffer[K210_Uart_Rx_Index - 1] != 0x5B))                  // ???2
        goto Send_Error;

    // ?ì2é?üá?êy?Y·??§ê?·?o?àí
    if ((K210_Uart_Rx_Index - 3) % 2)
        goto Send_Error;

    for (i = 0; i < ((K210_Uart_Rx_Index - 3) >> 1); ++i)
    {
        K210_Rx_Data[i] = K210_Uart_Rx_Buffer[2 + (i << 1) + 1] << 8; // ??8??
        K210_Rx_Data[i] |= K210_Uart_Rx_Buffer[2 + (i << 1) + 0];     // μí8??
    }

    goto Jump; // ?y3￡á÷3ì?íì?1y
Send_Error:    // ?a??′í?ó
Jump:
    // ??á??óê??o3? ?a??ò?′??óê?×?±?
    K210_Uart_Rx_Index = 0;
		for(int i=0;i<8;i++)
			K210_Uart_Rx_Buffer[i]=0;
		
		if(Num==0)
			Num=K210_Rx_Data[1];
		if(Load_Flag==0)
		{
			switch (Num)
			{
				case 0:
					Target_Room=0;
				break;
				case 1:
					Target_Room='A';
				break;
				case 2:
					Target_Room='B';
				break;
				default:
					Target_Room='G';
				break;
			}
		}
		LorR=K210_Rx_Data[0];
		if(K210_Uart_Tx_Buffer[4]==1)
		{
			if(LorR!=0)
				Far_Flag=1;
			else
				Far_Flag=2;
		}
	
		
}
void K210_Transmit(u8* Tx)
{
	for(int t=0;t<8;t++)
	{ 
		USART_SendData(UART4, Tx[t]); //向串口 1 发送数据
		while(USART_GetFlagStatus(UART4,USART_FLAG_TC)!=SET);
		//等待发送结束
	}
}