#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

#ifdef __cplusplus
	extern "C" {
#endif

//定义最大样本数
#define DELTA_MAX_SAMPLES 12

typedef struct 
{
    float Last_P;//上次估算协方差 不可以为0 ! ! ! ! ! 
    float Now_P;//当前估算协方差
    float out;//卡尔曼滤波器输出
    float Kg;//卡尔曼增益
    float Q;//过程噪声协方差
    float R;//观测噪声协方差
}Kalman;
extern Kalman kfp;
void Kalman_Init(Kalman *kfp1);
float KalmanFilter1_adc(Kalman *kfp,float input);
float KalmanFilter1(Kalman *kfp,float input);		
float Kalman_Filter_x(float Accel,float Gyro);		
float Complementary_Filter_x(float angle_m, float gyro_m);
float Kalman_Filter_y(float Accel,float Gyro);		
float Complementary_Filter_y(float angle_m, float gyro_m);
float Complementary_ADC(float ADC_m);
void Gyro_Fit(int way,float *GYRO);
void Acc_Fit(int way,float *aacx,float *aacy,float *aacz);


//低通滤波器的系数
typedef struct pt1Filter_s 
{
	float state;	//滤波结果
	float RC;			//模电RC滤波的电阻和电容值的积
	float dT;			//数据更新时间隔（秒）
}pt1Filter_t;

//IIR 滤波器的系数
typedef struct biquadFilter_s 
{
	float b0, b1, b2, a1, a2;
	float d1, d2;
}biquadFilter_t;

typedef enum 
{
	FILTER_PT1 = 0,
	FILTER_BIQUAD,
}filterType_e;

//滤波器的种类
typedef enum 
{
	FILTER_LPF,						//低通滤波器
	FILTER_NOTCH					//带通滤波器
}biquadFilterType_e;

typedef float (*filterApplyFnPtr)(void *filter, float input);
float nullFilterApply(void *filter, float input);
void pt1FilterInit(pt1Filter_t *filter, uint8_t f_cut, float dT);
float pt1FilterApply(pt1Filter_t *filter, float input);
float pt1FilterApply4(pt1Filter_t *filter, float input, uint8_t f_cut, float dT);


		
#ifdef __cplusplus
	}
#endif
		
#endif
