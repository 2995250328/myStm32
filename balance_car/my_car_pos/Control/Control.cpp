#include "Control.hpp"

/*
 * 函数功能：绝对值函数
 * 入口参数：a：需要计算绝对值的数
 * 返回  值：无符号整型
 */	
int myabs(int a)
{
	if(a<0)
		return 0-a;
	else 
		return a;
}
/*
 * 函数功能：限制PWM赋值 
 * 入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
 * 返回  值：限幅后的值
 */
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) 
		OUT = max;
	else if(OUT<min) 
		OUT = min;
	return OUT;
}
/*
 * 函数功能：赋值给PWM寄存器
 * 入口参数：左轮PWM、右轮PWM
 * 返回  值：无
 */
void Set_Pwm(int motor_left,int motor_right)
{
		if(motor_left>0) 
			BIN1=1, BIN2=0; //前进
		else if(motor_left<0)
			BIN1=0, BIN2=1; //后退
		else
			BIN1=0,BIN2=0;
		if(motor_right>0) 
			AIN1=1, AIN2=0; //前进
		else if(motor_right<0)
			AIN1=0, AIN2=1; //后退
		else
			AIN1=0,AIN2=0;
		PWMA=myabs(motor_right); //PWM 大小对应转速的大小
		PWMB=myabs(motor_left);
}
/*
 * 函数功能：异常关闭电机		
 * 入口参数：angle：小车倾角；voltage：电压
 * 返回  值：1：异常  0：正常
 */	
u8 Turn_Off(float angle, int voltage)
{
	if(angle<=-15||angle>=15||voltage<=11.7)
		return 1;
	else 
		return 0;
}
/*
 * 函数功能：编码器读数转换为速度（mm/s）
 * 入口参数：无
 * 返回  值：无
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	Velocity_Left=encoder_left*Control_Frequency/EncoderMultiples/Encoder_precision/Reduction_Ratio*Perimeter;//计算左轮速度
	Velocity_Right=encoder_right*Control_Frequency/EncoderMultiples/Encoder_precision/Reduction_Ratio*Perimeter;//计算右轮速度
}
/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID (int Position,int Target)
{
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=Target-Position; //计算偏差
	Integral_bias+=Bias; //求出偏差的积分
	//积分限幅
	if(Integral_bias>100000) Integral_bias=100000;
	if(Integral_bias<-100000) Integral_bias=-100000;
	Pwm=Position_KP*Bias+Position_KI*Integral_bias/10+Position_KD*(Bias-Last_Bias);
	//位置式 PID 控制器
	Last_Bias=Bias; //保存上一次偏差
	return Pwm; //增量输出
}
/*
 * 函数功能：增量（速度）PID控制	
 * 入口参数：编码器测量值，目标速度的单位时间编码器读数
 * 返回  值：电机控制的PWM
 */
int Incremental_PI (int Encoder,int Target)
{
	static float Bias,Pwm,Last_bias1,Last_bias2,Integral_bias;
	float yn;
	float alpha=0.2;//低通滤波系数
	Last_bias2=Last_bias1;
	Last_bias1=Bias;
	Bias=Target-Encoder; //计算偏差
	yn=Integral_bias;
	Integral_bias=Velocity_KI*Bias;
	Integral_bias=alpha*Integral_bias+(1-alpha)*yn;
	Pwm+=Velocity_KP*(Bias-Last_bias1)+Integral_bias+Velocity_KD*(Bias-2*Last_bias1+Last_bias2); //增量式 PID 控制器
	return Pwm; //增量输出
}