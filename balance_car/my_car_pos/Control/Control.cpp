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
 * ��ڲ�����angle��С����ǣ�voltage����ѹ
 * ����  ֵ��1���쳣  0������
 */	
u8 Turn_Off(float angle, int voltage)
{
	if(angle<=-15||angle>=15||voltage<=11.7)
		return 1;
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