#include "drv_Encoder_exti.hpp"

int counter_left=0,counter_right=0;

//�����ⲿ�ж϶�������
void Encoder_Init_Exit_Right(void)
{
	// PC6  ---> E1A  T3C1
	// PC7  ---> E1B  T3C2   
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
 
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI9_5_IRQn; //ѡ�񴥷���ͨ�� EXTI9_5_IRQn �� PC7 EXTI_Line7 һ��
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //��Ӧ���ȼ�
    NVIC_Init(&NVIC_InitStructure);
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;// ��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6|GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource7|GPIO_PinSource6); //����AFIO������ѡ��������ѡ������Ҫ�����ţ�ָ���ⲿ�ж���
 
    EXTI_InitStructure.EXTI_Line    = EXTI_Line7|EXTI_Line6;                  //��Ҫ���������ж���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                      //�����ж�
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;            //�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //���ô�����ʽ �����½��ش���
    EXTI_Init(&EXTI_InitStructure);                                //��������������Ϳ��Ը�������ṹ��
}

extern "C" {
void EXTI9_5_IRQHandler(void)    //������1���ⲿ�ж�
{
	EXTI_ClearITPendingBit(EXTI_Line7); //�ֶ�����ⲿ�жϱ�־λ 
	EXTI_ClearITPendingBit(EXTI_Line6); //�ֶ�����ⲿ�жϱ�־λ 
	if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //pc6������
	{	
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //�ж� encoder_pin_B ��ƽ״̬��
    {
			counter_right--;
    }    
    else
    {
			counter_right++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6))//pc6�½���
	{
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //�ж� encoder_pin_B ��ƽ״̬��
    {
			counter_right--;
    }    
    else
    {
			counter_right++;
    }
	}
	if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7)) //pc7������
	{	
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //�ж� encoder_pin_A ��ƽ״̬��
    {
			counter_right++;
    }    
    else
    {
			counter_right--;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_7))//pc7�½���
	{
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_6)) //�ж� encoder_pin_A ��ƽ״̬��
    {
			counter_right++;
    }    
    else
    {
			counter_right--;
    }
	}
}
}
int Encoder_Readnum_Right(void)
{		
    int result = counter_right;
		counter_right=0;
		return result;
}
//���ֱ������ⲿ�ж϶�ȡ
void Encoder_Init_Exit_Left(void)
{
		// PD12 ---> E2A  T4C1
		// PD13 ---> E2B  T4C2
		GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
 
    NVIC_InitStructure.NVIC_IRQChannel                   = EXTI15_10_IRQn; //ѡ�񴥷���ͨ�� EXTI15_10_IRQn 
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;     // ʹ��
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;          //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;          //��Ӧ���ȼ�
    NVIC_Init(&NVIC_InitStructure);
 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    
    GPIO_InitStructure.GPIO_Mode =GPIO_Mode_IN_FLOATING;// ��������
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12|GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
   
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOD, GPIO_PinSource12|GPIO_PinSource13); //����AFIO������ѡ��������ѡ������Ҫ�����ţ�ָ���ⲿ�ж���
 
    EXTI_InitStructure.EXTI_Line    = EXTI_Line12|EXTI_Line13;                  //��Ҫ���������ж���
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;                      //�����ж�
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;            //�ж�ģʽ
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //���ô�����ʽ �����½��ش���
    EXTI_Init(&EXTI_InitStructure);                                //��������������Ϳ��Ը�������ṹ��
}
extern "C" {
void EXTI15_10_IRQHandler(void)    //������2���ⲿ�ж�
{	
	if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12)) //pd12������
	{	
		if (0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //�ж� encoder_pin_B ��ƽ״̬��
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12))//pd12�½���
	{
		if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //�ж� encoder_pin_B ��ƽ״̬��
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if (1 == GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_13)) //pd13������
	{	
		if (1 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)) //�ж� encoder_pin_A ��ƽ״̬��
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
	else if(0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13))//pd13�½���
	{
		if (0 == GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12)) //�ж� encoder_pin_A ��ƽ״̬��
    {
			counter_left--;
    }    
    else
    {
			counter_left++;
    }
	}
		EXTI_ClearITPendingBit(EXTI_Line12); //�ֶ�����ⲿ�жϱ�־λ 
	EXTI_ClearITPendingBit(EXTI_Line13); //�ֶ�����ⲿ�жϱ�־λ 
}
}
int Encoder_Readnum_left(void)
{		
    int result = counter_left;
		counter_left=0;
		return result;
}