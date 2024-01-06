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
			PDout(4)=1, PCout(12)=0; //前进
		else if(motor_right<0)
			PDout(4)=0, PCout(12)=1; //后退
		else
			PDout(4)=0,PCout(12)=0;
		PWMA=myabs(motor_right); //PWM 大小对应转速的大小
		PWMB=myabs(motor_left);
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
 * 函数功能：设置前进固定距离	
 * 入口参数：目标距离
 * 返回  值：无
 */	
void Car_Go(int Target_Location)
{
	Straight_Flag=1;
	Stop_Flag=0;
	Spin_Flag=0;
	Target_Position_L=(float)((float)Target_Location*15600)/(float)Perimeter;
	Target_Position_R=(float)((float)Target_Location*15600)/(float)Perimeter;
	Location_CM_L=0;
	Location_CM_R=0;
	Position_L=0;
	Position_R=0;
}
/*
 * 函数功能：设置转向角度	
 * 入口参数：目标角度 枚举型
 * 返回  值：无
 */	
void Car_Spin(spin_dir_t Turn)
{
	Straight_Flag=0;
	Stop_Flag=0;
	Spin_Flag=1;
	Location_CM_L=0;
	Location_CM_R=0;
	Position_L=0;
	Position_R=0;
	float spin90_val;
	spin90_val=0.25*PI*Wheel_Track;
	spin90_val=(float)spin90_val*15600/Perimeter;
	if(Turn==left_90)
	{
		Target_Position_L=spin90_val*0.97;
		Target_Position_R=-spin90_val*0.97;
	}
	else if(Turn==right_90)
	{
		Target_Position_L=-spin90_val*0.97;
		Target_Position_R=spin90_val*0.97;
	}
	else if(Turn==back_180)
	{
		Target_Position_L=-2*spin90_val*0.94;
		Target_Position_R=2*spin90_val*0.94;
	}
	else if(Turn==num_spin1)
	{
		Target_Position_L=spin90_val*0.94*0.15;
		Target_Position_R=-spin90_val*0.94*0.15;
	}
	else if(Turn==num_spin2)
	{
		Target_Position_L=-spin90_val*0.94*0.15;
		Target_Position_R=spin90_val*0.94*0.15;
	}
}
/*
 * 函数功能：位置速度串级pid
 * 入口参数：编码器测量值转换为距离，目标位置
 * 返回  值：电机控制的PWM
 */
int Location_Speed_L(int Encoder,int Location,int Target_P,int Target_V)
{
	int Position_Moto;
	int limit_a;
	Position_Moto=Position_PID_L(Location,Target_P);
	limit_a=PWM_Limit(Position_Moto,Target_V,-Target_V);
	Position_Moto=Speed_PID_L(Encoder,limit_a);
	Position_Moto=PWM_Limit(Position_Moto,6700,-6700);
	return Position_Moto;
}
/*
 * 函数功能：位置速度串级pid
 * 入口参数：编码器测量值转换为的距离，目标位置
 * 返回  值：电机控制的PWM
 */
int Location_Speed_R(int Encoder,int Location,int Target_P,int Target_V)
{
	int Position_Moto1;
	int limit_a1;
	Position_Moto1=Position_PID_R(Location,Target_P);
	limit_a1=PWM_Limit(Position_Moto1,Target_V,-Target_V);
	Position_Moto1=Speed_PID_R(Encoder,limit_a1);
	Position_Moto1=PWM_Limit(Position_Moto1,6700,-6700);
	return Position_Moto1;
}
/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID_L (int Position,int Target)
{
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=Target-Position; //计算偏差
	Integral_bias+=Bias; //求出偏差的积分
	if(myabs(Bias)<0.5)
		Bias=0;
	//积分限幅
	if(Integral_bias>7000) Integral_bias=7000;
	if(Integral_bias<-7000) Integral_bias=-7000;
	if(Stop_Flag==1)
		Integral_bias=0;
	Pwm=Position_KP_L*Bias+Position_KI_L*Integral_bias+Position_KD_L*(Bias-Last_Bias);
	//位置式 PID 控制器
	Last_Bias=Bias; //保存上一次偏差
	return Pwm; //增量输出
}
/*
 * 函数功能：（速度）PID控制	
 * 入口参数：编码器测量值，目标速度的单位时间编码器读数
 * 返回  值：电机控制的PWM
 */
int Speed_PID_L (int Encoder,int Target)
{
	static float Bias,Pwm,Last_bias1,Last_bias2,Integral_bias;
	float yn;
	float alpha=0.2;//低通滤波系数
	Last_bias2=Last_bias1;
	Last_bias1=Bias;
	Bias=Target-Encoder; //计算偏差
	if(myabs(Bias)<1)
		Bias=0;
	yn=Integral_bias;
	Integral_bias=Velocity_KI_L*Bias;
	Integral_bias=alpha*Integral_bias+(1-alpha)*yn;
	if(Integral_bias>7000)
		Integral_bias=7000;
	else if(Integral_bias<-7000)
		Integral_bias=-7000;
	if(Stop_Flag==1)
		Integral_bias=0;
	Pwm+=Velocity_KP_L*(Bias-Last_bias1)+Integral_bias+Velocity_KD_L*(Bias-2*Last_bias1+Last_bias2); //增量式 PID 控制器
	return Pwm; //增量输出
}
/*
 * 函数功能：位置PID控制	
 * 入口参数：Positon 当前位置即编码器读数 Target 目标位置
 * 返回  值：电机控制的PWM
 */	
int Position_PID_R(int Position,int Target)
{
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=Target-Position; //计算偏差
	Integral_bias+=Bias; //求出偏差的积分
	//积分限幅
	if(Integral_bias>7000) Integral_bias=7000;
	if(Integral_bias<-7000) Integral_bias=-7000;
	if(Stop_Flag==1)
		Integral_bias=0;
	Pwm=Position_KP_R*Bias+Position_KI_R*Integral_bias+Position_KD_R*(Bias-Last_Bias);
	//位置式 PID 控制器
	Last_Bias=Bias; //保存上一次偏差
	return Pwm; //增量输出
}
/*
 * 函数功能：增量（速度）PID控制	
 * 入口参数：编码器测量值，目标速度的单位时间编码器读数
 * 返回  值：电机控制的PWM
 */
int Speed_PID_R(int Encoder,int Target)
{
	static float Bias1,Pwm1,Last_bias11,Last_bias21,Integral_bias1;
	float yn;
	float alpha=0.2;//低通滤波系数
	Last_bias21=Last_bias11;
	Last_bias11=Bias1;
	Bias1=Target-Encoder; //计算偏差
	if(myabs(Bias1)<0.5)
		Bias1=0;
	yn=Integral_bias1;
	Integral_bias1=Velocity_KI_R*Bias1;
	Integral_bias1=alpha*Integral_bias1+(1-alpha)*yn;
	if(Integral_bias1>7000)
		Integral_bias1=7000;
	else if(Integral_bias1<-7000)
		Integral_bias1=-7000;
	if(Stop_Flag==1)
		Integral_bias1=0;
	Pwm1+=Velocity_KP_R*(Bias1-Last_bias11)+Integral_bias1+Velocity_KD_R*(Bias1-2*Last_bias11+Last_bias21); //增量式 PID 控制器
	return Pwm1; //增量输出
}
float OPENMV_Turn_PID(int x_position)
{ 	                                         //??á?ê?3?
	 static float x_Bias,turn_Pwm,x_Integral_bias,x_Last_Bias;
	 float target=0;
	 x_Bias=target-x_position; 	//??????2?
	 x_Integral_bias+=x_Bias;		//?ó3???2?μ??y·
	if(Stop_Flag==1)
	{
		x_Integral_bias=0;
	}
	 if(x_Integral_bias>7000)
		 x_Integral_bias=7000;
	 if(x_Integral_bias<-7000)
		 x_Integral_bias=-7000;
	 turn_Pwm=OpenMv_KP*x_Bias+OpenMv_KI*x_Integral_bias+OpenMv_KD*(x_Bias-x_Last_Bias);       //????ê?PID?????÷
	 x_Last_Bias=x_Bias;  	//±￡′?é?ò?′???2? 
	 return turn_Pwm;  
}

