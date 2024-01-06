#include "Basic.hpp"
#include "drv_Blooth.hpp"
#include "Commulink.hpp"

/*
 * 函数功能：串口3初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart3(u32 bound)
{
	//GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//使能USART3，GPIOD时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	//重映射使能
	GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
	//USART3_TX   GPIOD.8
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //USART3_RX	  GPIOD.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	//USART3 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化NVIC寄存器
  //USART 初始化设置
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	
	//初始化串口3
  USART_Init(USART3, &USART_InitStructure); 
	//开启串口接受中断
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	//使能串口3 
  USART_Cmd(USART3, ENABLE);                    
}
extern u8 i;
int uart3_state;
extern u8 Blooth_Uart_Rx_Buffer[8];
extern u8 Blooth_Uart_Rx_Index;
extern u8 Blooth_Rx_Data[8];
extern "C"{
void USART3_IRQHandler(void)                	//串口1中断服务程序
{
//	PEout(0)=!PEout(0);
#if SYSTEM_SUPPORT_OS 		//è?1?SYSTEM_SUPPORT_OS?a??￡??òDèòa?§3?OS.
	OSIntEnter();    
#endif
		u8 com_data; 							
		static uint8_t RxCounter1=0;
		static uint8_t RxBuffer1[9]={0};		
		if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //?óê??D??(?óê?μ?μ?êy?Y±?D?ê?0x0d 0x0a?á?2)
		{
			com_data = USART_ReceiveData(USART3);
			if(uart3_state==0&&com_data==0x2C)
			{
				uart3_state=1;
				Blooth_Uart_Rx_Buffer[RxCounter1++]=com_data;//0
			}
			else if(uart3_state==1&&com_data==0x12)
			{
				uart3_state=2;
				Blooth_Uart_Rx_Buffer[RxCounter1++]=com_data; //1
			}				
			else if(uart3_state==2)
			{
					Blooth_Uart_Rx_Buffer[RxCounter1++]=com_data;
					if(RxCounter1>=9 || com_data == 0x5B)       //RxBuffer1????,??????
					{
				  uart3_state=3;
					}
			}
			else if(uart3_state==3)						
			{
				if(Blooth_Uart_Rx_Buffer[RxCounter1-1] == 0x5B)
				{
							
					RxCounter1 = 0;
					uart3_state = 0;
				//	Blooth_Process(RxBuffer1);
				}
				else   
				{
					uart3_state = 0;
					RxCounter1=0;
				}
			} 
			else  
			{
				uart3_state = 0;
				RxCounter1=0;
			}
					USART_ClearFlag(USART3,USART_FLAG_RXNE);//清除RXNE标志位
			USART_ClearITPendingBit(USART3,USART_FLAG_RXNE);
		}		
		i=com_data;
		  if (Blooth_Uart_Rx_Index < 50) // ????????
    {
        Blooth_Uart_Rx_Index=RxCounter1;                                            // ???1
    }
			if(USART_GetFlagStatus(USART3, USART_IT_ORE) != RESET)  
                    //需要用USART_GetFlagStatus函数来检查ORE溢出中断
		{
			USART_ClearFlag(USART3,USART_FLAG_ORE);//清除ORE标志位
			USART_ReceiveData(USART3);	           //抛弃接收到的数据			
     }
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
} 
}
void Blooth_Check_Data_Task(void)
{
    uint16_t i;
    if (!Blooth_Uart_Rx_Index) // ??óDêy?Y í?3?
		{    return;
		}
    // °′DèDT?? ￡¨7￡oá???short￡? 5￡oò???short￡?òà′?ààí?￡
    if (Blooth_Uart_Rx_Index < 7) // ?óê?3¤?èê?·?′?μ?
        return;

    // ?ì2é??ê?ê?·??yè·
    if ((Blooth_Uart_Rx_Buffer[0] != 0x2C) && (Blooth_Uart_Rx_Buffer[1] != 0x12) && // ??í·(á???×??ú)
        (Blooth_Uart_Rx_Buffer[Blooth_Uart_Rx_Index - 1] != 0x5B))                  // ???2
        goto Send_Error;

    // ?ì2é?üá?êy?Y·??§ê?·?o?àí
    if ((Blooth_Uart_Rx_Index - 3) % 2)
        goto Send_Error;

    for (i = 0; i < ((Blooth_Uart_Rx_Index - 3) >> 1); ++i)
    {
        Blooth_Rx_Data[i] = Blooth_Uart_Rx_Buffer[2 + (i << 1) + 1] << 8; // ??8??
        Blooth_Rx_Data[i] |= Blooth_Uart_Rx_Buffer[2 + (i << 1) + 0];     // μí8??
    }

    goto Jump; // ?y3￡á÷3ì?íì?1y
Send_Error:    // ?a??′í?ó
Jump:
    // ??á??óê??o3? ?a??ò?′??óê?×?±?
    Blooth_Uart_Rx_Index = 0;
		for(int i=0;i<8;i++)
			Blooth_Uart_Rx_Buffer[i]=0;	
}
void Blooth_Transmit(u8* Tx)
{
	for(int t=0;t<8;t++)
	{ 
		USART_SendData(USART3, Tx[t]); //向串口 1 发送数据
		while(USART_GetFlagStatus(USART3,USART_FLAG_TC)!=SET);
		//等待发送结束
	}
}