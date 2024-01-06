#include "drv_Key.hpp"
#include "Basic.hpp"
#include "delay.h"
#include "stm32f10x_exti.h"
//长按标识
int tran = 0;
static u16 Press_count;
extern char operation;//操作符变量
extern int operative_index;//操作符的索引
extern int number1[3];//第一个数字
extern int number2[3];//第二个数字
extern int indication;//位的指示值
extern int result;//存储运算结果
extern int num;

extern u16 n;

/**************************************************************************
函数功能：按键切换功能
入口参数：无
返回  值：无
加入硬件去抖不可使用长按双击检测
**************************************************************************/
void Key(void)
{	
	u16 temp1,temp2;
	u16 tmp,tmp2;
	
	#if( Is_Detrembling != 0 )
		tmp = click();
		//按键按下
		if(tmp==1)
		{
			GPIO_ResetBits(GPIOE,GPIO_Pin_0);
			return;
		}
		else{}
	#else
	tmp=click_N_Double(6);
	//单击
	if(tmp==1)
	{
		if(indication==0)//对第一个数的操作
		{
			if(number1[0]<9)
				number1[0]++;
			else 
				number1[0]=0;
		}
		else if(indication==1)
		{
			if(number1[1]<9)
				number1[1]++;
			else 
				number1[1]=0;
		}
		else if(indication==2)
		{
			if(number1[2]<9)
				number1[2]++;
			else 
				number1[2]=0;
		}
		else if(indication==3)//对运算符的变换
		{
			if(operative_index==0)
			{operation='+';
				operative_index++;}
			else if(operative_index==1)
			{operation='-';
				operative_index++;}
			else if(operative_index==2)
			{operation='*';
				operative_index++;}
			else if(operative_index==3)
			{operation='/';
				operative_index=0;}
		}
		else if(indication==4)//对第二个数的操作
		{
			if(number2[0]<9)
				number2[0]++;
			else 
				number2[0]=0;
		}
		else if(indication==5)
		{
			if(number2[1]<9)
				number2[1]++;
			else 
				number2[1]=0;
		}
		else if(indication==6)
		{
			if(number2[2]<9)
				number2[2]++;
			else 
				number2[2]=0;
		}
		return;
	}
	//双击
	else if(tmp==2)
	{
		if(indication<6)
			indication++;
		else 
			indication=0;
		return;
	} 
	tmp2=Long_Press();
	temp1=number1[0]+number1[1]*10+number1[2]*100;//将两个操作数转化为整数，进行运算
	temp2=number2[0]+number2[1]*10+number2[2]*100;
	//长按键发送
	if(tmp2==3)
	{
		n++;
		if(operation=='+')
			result=temp1+temp2;
		else if(operation=='-')
			result=temp1-temp2;
		else if(operation=='*')
			result=temp1*temp2;
		else if(operation=='/')
			result=temp1/temp2;
		return;
	}
	#endif	
}

/**************************************************************************
函数功能：按键扫描
入口参数：双击等待时间
返回  值：按键状态 0：无动作 1：单击 2：双击 
**************************************************************************/
u8 click_N_Double (u16 time)
{
	static	u16 flag_key=0,count_key=0,double_key=0;	
	static	u16 count_single=0,Forever_count=0;
	if(KEY==0)  							//KEY为PC0管脚状态，此时按键按下
		Forever_count++;   					//长按标志位未置1
    else       
		Forever_count=0;
	if(KEY == 0 && flag_key == 0)			//按键按下，且按键按下标志为0	
		flag_key=1;	
	if(count_key == 0)						//第一次为0
	{
		if(flag_key==1) 
		{
			double_key++;					//按键按下一次，double_key加一次
			count_key=1;					//按键按下，count=1
		}
		if(double_key==2) 
		{
			double_key=0;
			count_single=0;
			return 2;//双击执行的指令
		}
	}
	if(KEY == 1)			
		flag_key=0,count_key=0;				//按键未按下
	if(double_key == 1)						//按键已经按下一次
	{
		count_single++;						//超过等待时间
		if(count_single>time&&Forever_count<time)
		{
			double_key=0;
			count_single=0;	
			return 1;//单击执行的指令
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
函数功能：按键扫描
入口参数：无
返回  值：按键状态 0：无动作 1：单击 
**************************************************************************/
u8 click(void)
{
	static u8 flag_key=1;//按键按松开标志
	if(flag_key&&KEY==0)
	{
		flag_key=0;
		return 1;	// 按键按下
	}
	else if(1==KEY)			flag_key=1;
		return 0;//无按键按下
}
/**************************************************************************
函数功能：长按检测
入口参数：无
返回  值：按键状态 0：无动作 3：长按2s
**************************************************************************/
u8 Long_Press(void)
{
	static u16 Long_Press_count,Long_Press;
	if(Long_Press==0&&KEY==0) 
	{		
		Press_count++;
		Long_Press_count++;   //长按标志位未置1
	}
	else                       
		Long_Press_count=0; 
	if(Long_Press_count>25)		
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
	if(Long_Press==1)     //长按标志位置1
	{
		Long_Press=0;
	}
	return 0;
}

//Key执行函数
static void Key_Task(void* pvParameters)
{
	while(1)
	{
		Key();
		delay_ms(15);
	}
}

//按键初始化函数
void init_drv_Key(void)
{//KEY  PC0
	//开启GPIOC外设时钟,开启复用功能时钟
	RCC->APB2ENR|=(1<<4)|(1<<0);
	//KEY初始化
	os_delay(1e-2);
	//配置引脚输出模式
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);              //使能 PORTC 时钟 
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;                       //上拉输入
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0;                           //PC0 
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}

void EXTIX_Init(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	 init_drv_Key(); //①按键端口初始化
		
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE); //②开启 AFIO 时钟
	//GPIOC.0 中断线以及中断初始化配置,下降沿触发
		
	 GPIO_EXTILineConfig(GPIO_PortSourceGPIOC,GPIO_PinSource0);//③
	 EXTI_InitStructure.EXTI_Line=EXTI_Line0;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	 EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure); //④初始化中断线参数
 
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //使能按键外部中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00; //抢占优先级 2，
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x02; //子优先级 2
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //使能外部中断通道
	NVIC_Init(&NVIC_InitStructure);//⑤初始化 NVIC
}
//⑥外部中断 2 服务程序
extern"C"{
void EXTI0_IRQHandler(void)
{
	EXTI_ClearITPendingBit(EXTI_Line0); //清除 LINE2 上的中断标志位 
	if(KEY==0)
	{
		if(num==0)
			num++;
		else if(num==1)
			num++;
		else if(num==2)
			num=0;
	}
}}