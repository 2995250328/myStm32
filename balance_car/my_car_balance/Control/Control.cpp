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
 * 入口参数：angle：小车倾角；flag：电压flag
 * 返回  值：1：异常  0：正常
 */	
u8 Turn_Off(float angle, int flag)
{
	if(angle<=-30||angle>=30||flag==1)
	{
		AIN1=0;                                            
		AIN2=0;
		BIN1=0;
		BIN2=0;
		return 1;
	}
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
/*
 * 函数功能：检测小车是否被拿起
 * 入口参数：Acceleration：z轴加速度；Angle：平衡的角度；encoder_left：左编码器计数；encoder_right：右编码器计数
 * 返回  值：1:小车被拿起  0：小车未被拿起
 */
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //第一步
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //条件1，小车接近静止
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //进入第二步
	 {
			if(++count1>200)       count1=0,flag=0;                       //超时不再等待2000ms，返回第一步
			if(Acceleration>23000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //条件2，小车是在0度附近被拿起
			flag=2; 
	 } 
	 if(flag==2)                                                       //第三步
	 {
		  if(++count2>100)       count2=0,flag=0;                        //超时不再等待1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //条件3，小车的轮胎因为正反馈达到最大的转速   
      {
				flag=0;                                                                                     
				return 1;                                                    //检测到小车被拿起
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance；Left encoder count；Right encoder count
Output  : 1：put down  0：No action
函数功能：检测小车是否被放下
入口参数：平衡角度；左编码器读数；右编码器读数
返回  值：1：小车放下   0：小车未放下
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //防止误检      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //条件1，小车是在0度附近的
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //超时不再等待 250ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_right>3&&encoder_right<40) //条件2，小车的轮胎在未上电的时候被人为转动  
      {
				flag=0;
				count=0;
				return 1;                         //检测到小车被放下
			}
	 }
	return 0;
}
/*
 * 函数功能：直立PD控制		
 * 入口参数：Angle:角度；Gyro：角速度
 * 返回  值：balance：直立控制PWM
 */	
int Balance(float Angle,float Gyro)
{
	float Angle_bias,Gyro_bias;
	int balance;
  Angle_bias=Middle_angle-Angle;                       				//求出平衡的角度中值 和机械相关
	Gyro_bias=0-Gyro; 
	balance=-Balance_Kp*Angle_bias-Gyro_bias*Balance_Kd; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	return balance;
}
/*
 * 函数功能：速度控制PWM		
 * 入口参数：encoder_left：左轮编码器读数；encoder_right：右轮编码器读数
 * 返回  值：速度控制PWM
 */
int Velocity(int encoder_left,int encoder_right)
{
	static float velocity,Encoder_Least,Encoder_bias;
	static float Encoder_Integral;
	//=============速度 PI 控制器=======================//
	Encoder_Least =0-(encoder_left+encoder_right); //获取最新速度偏差=目标速度（此处为零）-测量速度（左右编码器之和）
	Encoder_bias *= 0.85; //一阶低通滤波器
	Encoder_bias+= Encoder_Least*0.15; //一阶低通滤波器
	//相当于上次偏差的 0.8 + 本次偏差的 0.2，减缓速度差值，减少对直立的干扰
	Encoder_Integral +=Encoder_bias; //积分出位移 积分时间：5ms
	if(Encoder_Integral>10000) Encoder_Integral=10000; //积分限幅
	if(Encoder_Integral<-10000) Encoder_Integral=-10000; //积分限幅
	velocity=-Encoder_bias*Velocity_Kp-Encoder_Integral*Velocity_Ki;//速度控制
	//电机关闭后清除积分
	if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) 
		Encoder_Integral=0;
	return velocity;
}
/*
 * 函数功能：转向控制 
 * 入口参数：偏航角 Z轴陀螺仪
 * 返回  值：转向控制PWM
 */
int Turn(float yaw,float gyro)
{
	static float turn,gyro_bias;
	Turn_Target=0;//根据转动角度不同改变这个值
	gyro_bias=Turn_Target-yaw;
	turn=-gyro_bias*Turn_Kp-gyro*Turn_Kd;//结合Z轴陀螺仪进行PD控制
	return turn;								 				
}