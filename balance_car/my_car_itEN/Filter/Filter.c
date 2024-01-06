#include "filter.h"
#include <math.h>
#include "drv_MPU6050.hpp"
#include <stdio.h>
#include "matrix.h"

//�����Ǿ�̬ƫ��
extern float gyro_offset[3];
//���ٶ�У׼
extern float acc_offset[3];
float dt=0.005;		  //ÿ5ms����һ���˲�  
Kalman kfp,k1,k2,k3,k4,k5,k6;
pt1Filter_t filter,f1,f2,f3,f4,f5,f6,f7,f8;
//һά�������˲�����ϵ���ĳ�ʼ��
void Kalman_Init(Kalman *kfp1)
{
	kfp1->Last_P = 1;			
	kfp1->Now_P = 0;		
	kfp1->out = 0;			
	kfp1->Kg = 0;		
	kfp1->Q = 0.0011;
	kfp1->R = 0.5;
}

/**
 *�������˲���
 *@param 	Kalman *kfp �������ṹ�����
 *   			float input ��Ҫ�˲��Ĳ����Ĳ���ֵ�����������Ĳɼ�ֵ��
 *@return �˲���Ĳ���������ֵ��
 */
float KalmanFilter1(Kalman *kfp,float input)//�����������ݵĿ������˲�
{
   //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);
   //����Э�����: ���ε�ϵͳЭ����� kfp->LastP Ϊ��һ������׼����
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
float KalmanFilter1_adc(Kalman *kfp,float input)//��adc����ֵ�Ŀ������˲�
{
   //Ԥ��Э����̣�kʱ��ϵͳ����Э���� = k-1ʱ�̵�ϵͳЭ���� + ��������Э����
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //���������淽�̣����������� = kʱ��ϵͳ����Э���� / ��kʱ��ϵͳ����Э���� + �۲�����Э���
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //��������ֵ���̣�kʱ��״̬����������ֵ = ״̬������Ԥ��ֵ + ���������� * ������ֵ - ״̬������Ԥ��ֵ��
   kfp->out = kfp->out  + kfp->Kg * (input -kfp->out);
   //����Э�����: ���ε�ϵͳЭ����� kfp->LastP Ϊ��һ������׼����
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
/**************************************************************************
�������ܣ��Զ�ȡ���Ľ��ٶ��������
��ڲ��������ٶȻ�ȡ�Ľ��ٶ� way:�˲���ʽ
����  ֵ����
**************************************************************************/
void Gyro_Fit(int way,float *GYRO)
{

		GYRO[0]-=gyro_offset[0];
		GYRO[1]-=gyro_offset[1];
		GYRO[2]-=gyro_offset[2];
	
}
/**************************************************************************
�������ܣ��Զ�ȡ���ļ��ٶ��������
��ڲ��������ٶȻ�ȡ�Ľ��ٶ� way:�˲���ʽ
����  ֵ����
**************************************************************************/
void Acc_Fit(int way,float *aacx,float *aacy,float *aacz)
{

		*aacx-=acc_offset[0];
		*aacy-=acc_offset[1];
		*aacz-=acc_offset[2];
	
}
/**************************************************************************
�������ܣ���ȡx��Ƕȼ��׿������˲�
��ڲ��������ٶȻ�ȡ�ĽǶȡ����ٶ�
����  ֵ��x��Ƕ�
**************************************************************************/               
float Kalman_Filter_x(float Accel,float Gyro)		
{
	//static float angle_dot;
	static float angle;
	float Q_angle=0.001; 		// ����������Э����
	float Q_gyro=0.003;			// ����������Э���� ����������Э����Ϊһ��һ�����о���
	float R_angle=0.5;			// ����������Э���� �Ȳ���ƫ��
	char  C_0 = 1;
	static float Q_bias, Angle_err;//Q_bias:�����ǵ�ƫ��  Angle_err:�Ƕ�ƫ��
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	//�״�Э�������
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; // �Ƕ�������ƻ��� ״̬����,�Ƕ�ֵ�����ϴ����ŽǶȼӽ��ٶȼ���Ư�����
	//Ԥ��Э�������ļ���
	/*
      PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0])*dt;
	    PP[0][1] = PP[0][1] - PP[1][1]*dt;
	    PP[1][0] = PP[1][0] - PP[1][1]*dt;
	    PP[1][1] = PP[1][1] + Q_gyro;
	*/
	Pdot[0]= - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt + Q_angle;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // ����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	// zk-������ƣ��Ƕ�ƫ��
	//�󿨶�������
	PCt_0 = C_0 * PP[0][0];//�󿨶�������ķ�ĸ
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;	// �󿨶��������ĸ
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;			// ���㿨��������
	//����Э�������
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	//���ÿ������������Ź���
	angle	+= K_0 * Angle_err;	 //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	//angle_dot   = Gyro - Q_bias; //���ֵ(�������)��΢��=���ٶ�
	return angle;
}

/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ��������ٶȻ�ȡ�ĽǶȡ����ٶ�
����  ֵ��x��Ƕ�
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}

/**************************************************************************
�������ܣ���ȡy��Ƕȼ��׿������˲�
��ڲ��������ٶȻ�ȡ�ĽǶȡ����ٶ�
����  ֵ��y��Ƕ�
**************************************************************************/
float Kalman_Filter_y(float Accel,float Gyro)		
{
	//static float angle_dot;
	static float angle;
	float Q_angle=0.001; 		// ����������Э����
	float Q_gyro=0.003;			// 0.003 ����������Э���� ����������Э����Ϊһ��һ�����о���
	float R_angle=0.5;			// ����������Э���� �Ȳ���ƫ��
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //�������
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-����������Э�����΢��
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-����������Э����΢�ֵĻ���
	PP[0][1] += Pdot[1] * dt;   // ����������Э����
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-�������
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //����������Э����
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	   //�������
	Q_bias	+= K_1 * Angle_err;	 //�������
	//angle_dot   = Gyro - Q_bias;	//���ֵ(�������)��΢��=���ٶ�
	return angle;
}

/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ��������ٶȻ�ȡ�ĽǶȡ����ٶ�
����  ֵ��y��Ƕ�
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}
/**************************************************************************
�������ܣ�һ�׻����˲�
��ڲ�����ADC����ֵ
����  ֵ���˲���Ĳ���ֵ
**************************************************************************/
float Complementary_ADC(float ADC_m)
{
	static float ADC;
	float K1=0.02;
	ADC= K1 * ADC_m+(1-K1)*ADC;
	return ADC;
}

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f
//���װ�����˹�˲����Ĵ���
#define BIQUAD_BANDWIDTH 1.9f           
//���װ�����˹�˲�����Ʒ������Q
#define BIQUAD_Q (1.0f / sqrtf(2.0f))   
#define UNUSED(x) (void)(x)

/*
 * �������ܣ��ͷ��˲���
 */
float nullFilterApply(void *filter, float input)
{
    UNUSED(filter);
    return input;
}

/*
 * �������ܣ���ͨ�˲���������
 * filter���˲�������ز���
 * f_cut����ֹƵ��
 * dT�����ݸ���ʱ������룩
 */
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
	//�ο�ģ��RC��ֹƵ�ʹ�ʽ
	filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
	//���ݸ���ʱ������룩
	filter->dT = dT;
}

/*
 * �������ܣ�Ӧ�õ�ͨ�˲�
 * filter���˲�������ز���
 * input�������������
 */
float pt1FilterApply(pt1Filter_t *filter, float input)
{
	//dt�̶�ʱ����ֹƵ��Խ�ͣ�kԽС��������������ı仯Խ������Ƶȥ���ˣ�
  //��ֹƵ�ʹ̶�ʱ��dTԽ��kԽС��������������ı仯Խ������Ƶȥ���ˣ�
	filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
	return filter->state;
}

/*
 * �������ܣ�Ӧ�õ�ͨ�˲�
 * filter���˲�������ز���
 * input�������������
 * f_cut����ֹƵ��
 */
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
	//��ǰ���㲢�������
	if (!filter->RC) 
	{
			pt1FilterInit(filter, f_cut, dT);
	}
	filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
	return filter->state;
}

