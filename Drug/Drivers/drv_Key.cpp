#include "drv_Key.hpp"
#include "Basic.hpp"
#include "delay.h"
#include "stm32f10x_exti.h"
//������ʶ
int tran = 0;
static u16 Press_count;
int way=1;//�����㷨�ı���
extern int Load_Flag;
extern int Delay_Flag;

/**************************************************************************
�������ܣ������л�����
��ڲ�������
����  ֵ����
����Ӳ��ȥ������ʹ�ó���˫�����
**************************************************************************/
void Key(void)
{	
	u16 tmp,tmp2;
	
	#if( Is_Detrembling != 0 )
		tmp = click();
		//��������
		if(tmp==1)
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_0);
			return;
		}
		else{}
	#else
	tmp=click_N_Double(10);
	//����
	if(tmp==1)
	{
		
		return;
	}
	//˫��
	else if(tmp==2)
	{
		
		return;
	} 
	tmp2=Long_Press();

	//����������
	if(tmp2==3)
	{
		
		return;
	}
	#endif	
}

/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫�� 
**************************************************************************/
u8 click_N_Double (u16 time)
{
	static	u16 flag_key=0,count_key=0,double_key=0;	
	static	u16 count_single=0,Forever_count=0;
	if(KEY==0)  							//KEYΪPC0�ܽ�״̬����ʱ��������
		Forever_count++;   					//������־λδ��1
    else       
		Forever_count=0;
	if(KEY == 0 && flag_key == 0)			//�������£��Ұ������±�־Ϊ0	
		flag_key=1;	
	if(count_key == 0)						//��һ��Ϊ0
	{
		if(flag_key==1) 
		{
			double_key++;					//��������һ�Σ�double_key��һ��
			count_key=1;					//�������£�count=1
		}
		if(double_key==2) 
		{
			double_key=0;
			count_single=0;
			return 2;//˫��ִ�е�ָ��
		}
	}
	if(KEY == 1)			
		flag_key=0,count_key=0;				//����δ����
	if(double_key == 1)						//�����Ѿ�����һ��
	{
		count_single++;						//�����ȴ�ʱ��
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	
			return 1;//����ִ�е�ָ��
		}
		if(Forever_count>time)
		{
			double_key=0;
			count_single=0;	
		}
	}	
	return 0;
}
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
	static u8 flag_key=1;//�������ɿ���־
	if(flag_key&&KEY==0)
	{
		flag_key=0;
		return 1;	// ��������
	}
	else if(1==KEY)			flag_key=1;
		return 0;//�ް�������
}
/**************************************************************************
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 3������2s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;
	if(Long_Press==0&&KEY==0) 
	{		
		Press_count++;
		Long_Press_count++;   //������־λδ��1
	}
	else                       
		Long_Press_count=0; 
	if(Long_Press_count>70)		
	{
		Long_Press=1;
		Long_Press_count=0;
		return 3;
	}	
	else if(Long_Press_count==1)
	{
		Press_count=0;
		return 1;
	}		
	if(Long_Press==1)     //������־λ��1
	{
		Long_Press=0;
	}
	return 0;
}

//Keyִ�к���
static void Key_Task(void* pvParameters)
{
	while(1)
	{
		Key();
		delay_ms(15);
	}
}

//������ʼ������
void init_drv_Key(void)
{//KEY  PC0
	//����GPIOC����ʱ��,�������ù���ʱ��
	RCC->APB2ENR|=(1<<4)|(1<<0);
	//KEY��ʼ��
	os_delay(1e-2);
	//�����������ģʽ
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);              //ʹ�� PORTC ʱ�� 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;                       //��������
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;                           //PC0 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	 init_drv_Key(); //�ٰ����˿ڳ�ʼ��
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //�ڿ��� AFIO ʱ��
	//GPIOC.0 �ж����Լ��жϳ�ʼ������,�½��ش���
		
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);//��
	 EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش���
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure); //�ܳ�ʼ���ж��߲���
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //ʹ�ܰ����ⲿ�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01; //��ռ���ȼ� 1��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //�����ȼ� 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //ʹ���ⲿ�ж�ͨ��
	NVIC_Init(&NVIC_InitStructure);//�ݳ�ʼ�� NVIC
}
//���ⲿ�ж� 2 �������
extern"C"{
void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0); //��� LINE2 �ϵ��жϱ�־λ 
	if(KEY==0)       
	{
		if(Load_Flag==0)
			Load_Flag=1;
		else if(Load_Flag==1)
			Load_Flag=2;
		Delay_Flag=1;
	}
}
}