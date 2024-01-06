#include "filter.h"
#include <math.h>
#include "drv_MPU6050.hpp"
#include <stdio.h>
#include "matrix.h"

//陀螺仪静态偏差
extern float gyro_offset[3];
//加速度校准
extern float acc_offset[3];
float dt=0.005;		  //每5ms进行一次滤波  
Kalman kfp,k1,k2,k3,k4,k5,k6;
pt1Filter_t filter,f1,f2,f3,f4,f5,f6,f7,f8;
//一维卡尔曼滤波各项系数的初始化
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
 *卡尔曼滤波器
 *@param 	Kalman *kfp 卡尔曼结构体参数
 *   			float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float KalmanFilter1(Kalman *kfp,float input)//对陀螺仪数据的卡尔曼滤波
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out + kfp->Kg * (input -kfp->out);
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
float KalmanFilter1_adc(Kalman *kfp,float input)//对adc采样值的卡尔曼滤波
{
   //预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差
   kfp->Now_P = kfp->Last_P + kfp->Q;
   //卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）
   kfp->Kg = kfp->Now_P / (kfp->Now_P + kfp->R);
   //更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）
   kfp->out = kfp->out  + kfp->Kg * (input -kfp->out);
   //更新协方差方程: 本次的系统协方差付给 kfp->LastP 为下一次运算准备。
   kfp->Last_P = (1-kfp->Kg) * kfp->Now_P;
   return kfp->out;
}
/**************************************************************************
函数功能：对读取到的角速度数据拟合
入口参数：加速度获取的角速度 way:滤波方式
返回  值：无
**************************************************************************/
void Gyro_Fit(int way,float *GYRO)
{

		GYRO[0]-=gyro_offset[0];
		GYRO[1]-=gyro_offset[1];
		GYRO[2]-=gyro_offset[2];
	
}
/**************************************************************************
函数功能：对读取到的加速度数据拟合
入口参数：加速度获取的角速度 way:滤波方式
返回  值：无
**************************************************************************/
void Acc_Fit(int way,float *aacx,float *aacy,float *aacz)
{

		*aacx-=acc_offset[0];
		*aacy-=acc_offset[1];
		*aacz-=acc_offset[2];
	
}
/**************************************************************************
函数功能：获取x轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角度
**************************************************************************/               
float Kalman_Filter_x(float Accel,float Gyro)		
{
	//static float angle_dot;
	static float angle;
	float Q_angle=0.001; 		// 过程噪声的协方差
	float Q_gyro=0.003;			// 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=0.5;			// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;//Q_bias:陀螺仪的偏差  Angle_err:角度偏量
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	//首次协方差矩阵
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; // 角度先验估计积分 状态方程,角度值等于上次最优角度加角速度减零漂后积分
	//预测协方差矩阵的计算
	/*
      PP[0][0] = PP[0][0] + Q_angle - (PP[0][1] + PP[1][0])*dt;
	    PP[0][1] = PP[0][1] - PP[1][1]*dt;
	    PP[1][0] = PP[1][0] - PP[1][1]*dt;
	    PP[1][1] = PP[1][1] + Q_gyro;
	*/
	Pdot[0]= - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt + Q_angle;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // 先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	// zk-先验估计，角度偏差
	//求卡尔曼增益
	PCt_0 = C_0 * PP[0][0];//求卡尔曼增益的分母
	PCt_1 = C_0 * PP[1][0];
	E = R_angle + C_0 * PCt_0;	// 求卡尔曼增益分母
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;			// 计算卡尔曼增益
	//更新协方差矩阵
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];
	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
	//利用卡尔曼进行最优估计
	angle	+= K_0 * Angle_err;	 //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	//angle_dot   = Gyro - Q_bias; //输出值(后验估计)的微分=角速度
	return angle;
}

/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：x轴角度
**************************************************************************/
float Complementary_Filter_x(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}

/**************************************************************************
函数功能：获取y轴角度简易卡尔曼滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角度
**************************************************************************/
float Kalman_Filter_y(float Accel,float Gyro)		
{
	//static float angle_dot;
	static float angle;
	float Q_angle=0.001; 		// 过程噪声的协方差
	float Q_gyro=0.003;			// 0.003 过程噪声的协方差 过程噪声的协方差为一个一行两列矩阵
	float R_angle=0.5;			// 测量噪声的协方差 既测量偏差
	char  C_0 = 1;
	static float Q_bias, Angle_err;
	static float PCt_0, PCt_1, E;
	static float K_0, K_1, t_0, t_1;
	static float Pdot[4] ={0,0,0,0};
	static float PP[2][2] = { { 1, 0 },{ 0, 1 } };
	angle+=(Gyro - Q_bias) * dt; //先验估计
	Pdot[0]=Q_angle - PP[0][1] - PP[1][0]; // Pk-先验估计误差协方差的微分
	Pdot[1]=-PP[1][1];
	Pdot[2]=-PP[1][1];
	Pdot[3]=Q_gyro;
	PP[0][0] += Pdot[0] * dt;   // Pk-先验估计误差协方差微分的积分
	PP[0][1] += Pdot[1] * dt;   // 先验估计误差协方差
	PP[1][0] += Pdot[2] * dt;
	PP[1][1] += Pdot[3] * dt;
	Angle_err = Accel - angle;	//zk-先验估计
	
	PCt_0 = C_0 * PP[0][0];
	PCt_1 = C_0 * PP[1][0];
	
	E = R_angle + C_0 * PCt_0;
	
	K_0 = PCt_0 / E;
	K_1 = PCt_1 / E;
	
	t_0 = PCt_0;
	t_1 = C_0 * PP[0][1];

	PP[0][0] -= K_0 * t_0;		 //后验估计误差协方差
	PP[0][1] -= K_0 * t_1;
	PP[1][0] -= K_1 * t_0;
	PP[1][1] -= K_1 * t_1;
		
	angle	+= K_0 * Angle_err;	   //后验估计
	Q_bias	+= K_1 * Angle_err;	 //后验估计
	//angle_dot   = Gyro - Q_bias;	//输出值(后验估计)的微分=角速度
	return angle;
}

/**************************************************************************
函数功能：一阶互补滤波
入口参数：加速度获取的角度、角速度
返回  值：y轴角度
**************************************************************************/
float Complementary_Filter_y(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.02; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}
/**************************************************************************
函数功能：一阶互补滤波
入口参数：ADC采样值
返回  值：滤波后的采样值
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
//二阶巴特沃斯滤波器的带宽
#define BIQUAD_BANDWIDTH 1.9f           
//二阶巴特沃斯滤波器的品质因数Q
#define BIQUAD_Q (1.0f / sqrtf(2.0f))   
#define UNUSED(x) (void)(x)

/*
 * 函数功能：释放滤波器
 */
float nullFilterApply(void *filter, float input)
{
    UNUSED(filter);
    return input;
}

/*
 * 函数功能：低通滤波器的设置
 * filter：滤波器的相关参数
 * f_cut：截止频率
 * dT：数据更新时间隔（秒）
 */
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT)
{
	//参考模电RC截止频率公式
	filter->RC = 1.0f / ( 2.0f * M_PI_FLOAT * f_cut );
	//数据更新时间隔（秒）
	filter->dT = dT;
}

/*
 * 函数功能：应用低通滤波
 * filter：滤波器的相关参数
 * input：新输入的数据
 */
float pt1FilterApply(pt1Filter_t *filter, float input)
{
	//dt固定时，截止频率越低，k越小，数据最终输出的变化越慢（高频去掉了）
  //截止频率固定时，dT越大，k越小，数据最终输出的变化越慢（高频去掉了）
	filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
	return filter->state;
}

/*
 * 函数功能：应用低通滤波
 * filter：滤波器的相关参数
 * input：新输入的数据
 * f_cut：截止频率
 */
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT)
{
	//先前计算并储存参数
	if (!filter->RC) 
	{
			pt1FilterInit(filter, f_cut, dT);
	}
	filter->state = filter->state + filter->dT / (filter->RC + filter->dT) * (input - filter->state);
	return filter->state;
}

