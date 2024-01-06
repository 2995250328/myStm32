#include "Basic.hpp"
#include "drv_OpenMv.hpp"
#include "Commulink.hpp"
#include "drv_MPU6050.hpp"
#include "Control.hpp"	

u8 Uart1_Receive;
/*
 * 函数功能：串口1初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart1(u32 bound)
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
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级3
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

/*
 * 函数功能：串口1接收中断
 * 入口参数：无
 * 返回  值：无
 */
extern u8 i;
int uart1_state;
extern u8 OpenMv_Uart_Rx_Buffer[8];
extern u8 OpenMv_Uart_Rx_Index;
extern u8 OpenMv_Rx_Data[8];
extern "C"{
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
//	PEout(0)=!PEout(0);
#if SYSTEM_SUPPORT_OS 		//è?1?SYSTEM_SUPPORT_OS?a??￡??òDèòa?§3?OS.
	OSIntEnter();    
#endif
		u8 com_data; 							
		static uint8_t RxCounter1=0;
		static uint8_t RxBuffer1[9]={0};		
		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //?óê??D??(?óê?μ?μ?êy?Y±?D?ê?0x0d 0x0a?á?2)
		{
			com_data = USART_ReceiveData(USART1);
			if(uart1_state==0&&com_data==0x2C)
			{
				uart1_state=1;
				OpenMv_Uart_Rx_Buffer[RxCounter1++]=com_data;//0
			}
			else if(uart1_state==1&&com_data==0x12)
			{
				uart1_state=2;
				OpenMv_Uart_Rx_Buffer[RxCounter1++]=com_data; //1
			}				
			else if(uart1_state==2)
			{
					OpenMv_Uart_Rx_Buffer[RxCounter1++]=com_data;
					if(RxCounter1>=9 || com_data == 0x5B)       //RxBuffer1????,??????
					{
				  uart1_state=3;
					}
			}
			else if(uart1_state==3)						
			{
				if(OpenMv_Uart_Rx_Buffer[RxCounter1-1] == 0x5B)
				{
							
					RxCounter1 = 0;
					uart1_state = 0;
				}
				else   
				{
					uart1_state = 0;
					RxCounter1=0;
				}
			} 
			else  
			{
				uart1_state = 0;
				RxCounter1=0;
			}
			USART_ClearFlag(USART1,USART_FLAG_RXNE);//清除RXNE标志位
			USART_ClearITPendingBit(USART1,USART_FLAG_RXNE);
		}		
		i=com_data;
		if (OpenMv_Uart_Rx_Index < 50) // ????????
    {
        OpenMv_Uart_Rx_Index=RxCounter1;                                            // ???1
    }
		if(USART_GetFlagStatus(USART1, USART_IT_ORE) != RESET)  
                    //需要用USART_GetFlagStatus函数来检查ORE溢出中断
		{
			USART_ClearFlag(USART1,USART_FLAG_ORE);//清除ORE标志位
			USART_ReceiveData(USART1);	           //抛弃接收到的数据			
     }
#if SYSTEM_SUPPORT_OS 	
	OSIntExit();  											 
#endif
} 
}
void OpenMv_Check_Data_Task(void)
{
    uint16_t i;
    if (!OpenMv_Uart_Rx_Index) // ??óDêy?Y í?3?
		{    return;
		}
    // °′DèDT?? ￡¨7￡oá???short￡? 5￡oò???short￡?òà′?ààí?￡
    if (OpenMv_Uart_Rx_Index < 7) // ?óê?3¤?èê?·?′?μ?
        return;

    // ?ì2é??ê?ê?·??yè·
    if ((OpenMv_Uart_Rx_Buffer[0] != 0x2C) && (OpenMv_Uart_Rx_Buffer[1] != 0x12) && // ??í·(á???×??ú)
        (OpenMv_Uart_Rx_Buffer[OpenMv_Uart_Rx_Index - 1] != 0x5B))                  // ???2
        goto Send_Error;

    // ?ì2é?üá?êy?Y·??§ê?·?o?àí
    if ((OpenMv_Uart_Rx_Index - 3) % 2)
        goto Send_Error;

    for (i = 0; i < ((OpenMv_Uart_Rx_Index - 3) >> 1); ++i)
    {
        OpenMv_Rx_Data[i] = OpenMv_Uart_Rx_Buffer[2 + (i << 1) + 1] << 8; // ??8??
        OpenMv_Rx_Data[i] |= OpenMv_Uart_Rx_Buffer[2 + (i << 1) + 0];     // μí8??
    }

    goto Jump; // ?y3￡á÷3ì?íì?1y
Send_Error:    // ?a??′í?ó
Jump:
    // ??á??óê??o3? ?a??ò?′??óê?×?±?
    OpenMv_Uart_Rx_Index = 0;
		for(int i=0;i<8;i++)
			OpenMv_Uart_Rx_Buffer[i]=0;
		
}