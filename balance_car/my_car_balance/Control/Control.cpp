#include "Control.hpp"

/*
 * �������ܣ�����ֵ����
 * ��ڲ�����a����Ҫ�������ֵ����
 * ����  ֵ���޷�������
 */	
int myabs(int a)
{
	if(a<0)
		return 0-a;
	else 
		return a;
}
/*
 * �������ܣ�����PWM��ֵ 
 * ��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
 * ����  ֵ���޷����ֵ
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
 * �������ܣ���ֵ��PWM�Ĵ���
 * ��ڲ���������PWM������PWM
 * ����  ֵ����
 */
void Set_Pwm(int motor_left,int motor_right)
{
		if(motor_left>0) 
			BIN1=1, BIN2=0; //ǰ��
		else if(motor_left<0)
			BIN1=0, BIN2=1; //����
		else
			BIN1=0,BIN2=0;
		if(motor_right>0) 
			AIN1=1, AIN2=0; //ǰ��
		else if(motor_right<0)
			AIN1=0, AIN2=1; //����
		else
			AIN1=0,AIN2=0;
		PWMA=myabs(motor_right); //PWM ��С��Ӧת�ٵĴ�С
		PWMB=myabs(motor_left);
}
/*
 * �������ܣ��쳣�رյ��		
 * ��ڲ�����angle��С����ǣ�flag����ѹflag
 * ����  ֵ��1���쳣  0������
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
 * �������ܣ�����������ת��Ϊ�ٶȣ�mm/s��
 * ��ڲ�������
 * ����  ֵ����
 */
void Get_Velocity_Form_Encoder(int encoder_left,int encoder_right)
{
	Velocity_Left=encoder_left*Control_Frequency/EncoderMultiples/Encoder_precision/Reduction_Ratio*Perimeter;//���������ٶ�
	Velocity_Right=encoder_right*Control_Frequency/EncoderMultiples/Encoder_precision/Reduction_Ratio*Perimeter;//���������ٶ�
}
/*
 * �������ܣ�λ��PID����	
 * ��ڲ�����Positon ��ǰλ�ü����������� Target Ŀ��λ��
 * ����  ֵ��������Ƶ�PWM
 */	
int Position_PID (int Position,int Target)
{
	static float Bias,Pwm,Integral_bias,Last_Bias;
	Bias=Target-Position; //����ƫ��
	Integral_bias+=Bias; //���ƫ��Ļ���
	//�����޷�
	if(Integral_bias>100000) Integral_bias=100000;
	if(Integral_bias<-100000) Integral_bias=-100000;
	Pwm=Position_KP*Bias+Position_KI*Integral_bias/10+Position_KD*(Bias-Last_Bias);
	//λ��ʽ PID ������
	Last_Bias=Bias; //������һ��ƫ��
	return Pwm; //�������
}
/*
 * �������ܣ��������ٶȣ�PID����	
 * ��ڲ���������������ֵ��Ŀ���ٶȵĵ�λʱ�����������
 * ����  ֵ��������Ƶ�PWM
 */
int Incremental_PI (int Encoder,int Target)
{
	static float Bias,Pwm,Last_bias1,Last_bias2,Integral_bias;
	float yn;
	float alpha=0.2;//��ͨ�˲�ϵ��
	Last_bias2=Last_bias1;
	Last_bias1=Bias;
	Bias=Target-Encoder; //����ƫ��
	yn=Integral_bias;
	Integral_bias=Velocity_KI*Bias;
	Integral_bias=alpha*Integral_bias+(1-alpha)*yn;
	Pwm+=Velocity_KP*(Bias-Last_bias1)+Integral_bias+Velocity_KD*(Bias-2*Last_bias1+Last_bias2); //����ʽ PID ������
	return Pwm; //�������
}
/*
 * �������ܣ����С���Ƿ�����
 * ��ڲ�����Acceleration��z����ٶȣ�Angle��ƽ��ĽǶȣ�encoder_left���������������encoder_right���ұ���������
 * ����  ֵ��1:С��������  0��С��δ������
 */
int Pick_Up(float Acceleration,float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count0,count1,count2;
	 if(flag==0)                                                      //��һ��
	 {
			if(myabs(encoder_left)+myabs(encoder_right)<30)               //����1��С���ӽ���ֹ
			count0++;
			else 
			count0=0;		
			if(count0>10)				
			flag=1,count0=0; 
	 } 
	 if(flag==1)                                                      //����ڶ���
	 {
			if(++count1>200)       count1=0,flag=0;                       //��ʱ���ٵȴ�2000ms�����ص�һ��
			if(Acceleration>23000&&(Angle>(-20+Middle_angle))&&(Angle<(20+Middle_angle)))   //����2��С������0�ȸ���������
			flag=2; 
	 } 
	 if(flag==2)                                                       //������
	 {
		  if(++count2>100)       count2=0,flag=0;                        //��ʱ���ٵȴ�1000ms
	    if(myabs(encoder_left+encoder_right)>70)                       //����3��С������̥��Ϊ�������ﵽ����ת��   
      {
				flag=0;                                                                                     
				return 1;                                                    //��⵽С��������
			}
	 }
	return 0;
}
/**************************************************************************
Function: Check whether the car is lowered
Input   : The angle of balance��Left encoder count��Right encoder count
Output  : 1��put down  0��No action
�������ܣ����С���Ƿ񱻷���
��ڲ�����ƽ��Ƕȣ���������������ұ���������
����  ֵ��1��С������   0��С��δ����
**************************************************************************/
int Put_Down(float Angle,int encoder_left,int encoder_right)
{ 		   
	 static u16 flag,count;	 
	 if(Flag_Stop==0)                     //��ֹ���      
			return 0;	                 
	 if(flag==0)                                               
	 {
			if(Angle>(-10+Middle_angle)&&Angle<(10+Middle_angle)&&encoder_left==0&&encoder_right==0) //����1��С������0�ȸ�����
			flag=1; 
	 } 
	 if(flag==1)                                               
	 {
		  if(++count>50)                     //��ʱ���ٵȴ� 250ms
		  {
				count=0;flag=0;
		  }
	    if(encoder_right>3&&encoder_right<40) //����2��С������̥��δ�ϵ��ʱ����Ϊת��  
      {
				flag=0;
				count=0;
				return 1;                         //��⵽С��������
			}
	 }
	return 0;
}
/*
 * �������ܣ�ֱ��PD����		
 * ��ڲ�����Angle:�Ƕȣ�Gyro�����ٶ�
 * ����  ֵ��balance��ֱ������PWM
 */	
int Balance(float Angle,float Gyro)
{
	float Angle_bias,Gyro_bias;
	int balance;
  Angle_bias=Middle_angle-Angle;                       				//���ƽ��ĽǶ���ֵ �ͻ�е���
	Gyro_bias=0-Gyro; 
	balance=-Balance_Kp*Angle_bias-Gyro_bias*Balance_Kd; //����ƽ����Ƶĵ��PWM  PD����   kp��Pϵ�� kd��Dϵ�� 
	return balance;
}
/*
 * �������ܣ��ٶȿ���PWM		
 * ��ڲ�����encoder_left�����ֱ�����������encoder_right�����ֱ���������
 * ����  ֵ���ٶȿ���PWM
 */
int Velocity(int encoder_left,int encoder_right)
{
	static float velocity,Encoder_Least,Encoder_bias;
	static float Encoder_Integral;
	//=============�ٶ� PI ������=======================//
	Encoder_Least =0-(encoder_left+encoder_right); //��ȡ�����ٶ�ƫ��=Ŀ���ٶȣ��˴�Ϊ�㣩-�����ٶȣ����ұ�����֮�ͣ�
	Encoder_bias *= 0.85; //һ�׵�ͨ�˲���
	Encoder_bias+= Encoder_Least*0.15; //һ�׵�ͨ�˲���
	//�൱���ϴ�ƫ��� 0.8 + ����ƫ��� 0.2�������ٶȲ�ֵ�����ٶ�ֱ���ĸ���
	Encoder_Integral +=Encoder_bias; //���ֳ�λ�� ����ʱ�䣺5ms
	if(Encoder_Integral>10000) Encoder_Integral=10000; //�����޷�
	if(Encoder_Integral<-10000) Encoder_Integral=-10000; //�����޷�
	velocity=-Encoder_bias*Velocity_Kp-Encoder_Integral*Velocity_Ki;//�ٶȿ���
	//����رպ��������
	if(Turn_Off(Angle_Balance,Voltage)==1||Flag_Stop==1) 
		Encoder_Integral=0;
	return velocity;
}
/*
 * �������ܣ�ת����� 
 * ��ڲ�����ƫ���� Z��������
 * ����  ֵ��ת�����PWM
 */
int Turn(float yaw,float gyro)
{
	static float turn,gyro_bias;
	Turn_Target=0;//����ת���ǶȲ�ͬ�ı����ֵ
	gyro_bias=Turn_Target-yaw;
	turn=-gyro_bias*Turn_Kp-gyro*Turn_Kd;//���Z�������ǽ���PD����
	return turn;								 				
}