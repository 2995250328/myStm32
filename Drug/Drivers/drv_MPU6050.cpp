#include "Basic.hpp"
#include "drv_MPU6050.hpp"
#include "drv_ExtIIC.hpp"
#include "Commulink.hpp"
#include "inv_mpu.h"
#include "filter.h"
#include <math.h>
#include "delay.h"
#include "Control.hpp"
#include <limits>
/*
备注：IIC_Send_Byte((MPU_ADDR<<1)|0);此命令中左移一位是为了留出一位控制位 1读 0写
	Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);左移三位是因为该寄存器低三位没有数据 要将数据左移三位才能存到指定位
	*gx=((u16)buf[0]<<8)|buf[1];  
	*gy=((u16)buf[2]<<8)|buf[3];  
	*gz=((u16)buf[4]<<8)|buf[5];高八位与低八位
*/
using namespace std;

//陀螺仪静态偏移量
float gyro_offset[3]={0};
//加速度校准
float acc_offset[3];
//陀螺仪变量
short gyro[3], accel[3], sensors;
//角速度值
float GYRO[3]={0};
//加速度值
float ACC[3]={0};
//三轴角度 Z轴陀螺仪
float Pitch,Roll,Yaw,Gryo_Z; 
//加速度传感器原始数据
float aacx,aacy,aacz;	
//陀螺仪原始数据
short gyrox,gyroy,gyroz;	

int16_t accRaw[3], gyroRaw[3];
int16_t temperuteRaw;
static uint8_t _mpu_half_resolution;
extern pt1Filter_t filter,f1,f2,f3,f4,f5,f6;
extern Kalman kfp,k1,k2,k3,k4,k5,k6;

/* IIC操作 */
	
	/*
	 * 函数功能：IIC写一个字节 
	 * reg：寄存器地址
	 * data：数据
	 * 返回值:0,正常 其他,错误代码
	 */
	u8 Write_Byte(u8 reg,u8 data) 				 
	{ 
	
		IIC_Start(); 
		IIC_Delay();
		//发送器件地址+写命令	
		IIC_Send_Byte((MPU_ADDR<<1)|0);
		//等待应答
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();		 
			return 1;		
		}
		//写寄存器地址
		IIC_Send_Byte(reg);	
		//等待应答 
		IIC_Wait_Ack();		
		//发送数据
		IIC_Send_Byte(data);
		//等待ACK
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();	 
			return 1;		 
		}		 
		IIC_Stop();	 
		return 0;
	}		
	/*
	 * 函数功能：IIC读一个字节 
	 * reg：寄存器地址
	 * 返回值:读到的数据
	 */
	u8 Read_Byte(u8 reg)
	{
		u8 res;
		IIC_Start(); 
		//发送器件地址+写命令	
		IIC_Send_Byte((MPU_ADDR<<1)|0);
		//等待应答 
		IIC_Wait_Ack();		
		//写寄存器地址
		IIC_Send_Byte(reg);	
		//等待应答
		IIC_Wait_Ack();		
		IIC_Start();
		//发送器件地址+读命令	
		IIC_Send_Byte((MPU_ADDR<<1)|1);
		//等待应答
		IIC_Wait_Ack();		 
		//读取数据,发送nACK 
		res=IIC_Read_Byte(0);
		//产生一个停止条件 
		IIC_Stop();			
		return res;		
	}

/* IIC操作 */

/* MPU6050操作 */
	
	/*
	 * 函数功能：初始化MPU6050
	 * 返回值:0,成功 其他,错误代码
	 */
	u8 MPU_Init(void)
	{ 
		u8 res;
		//初始化IIC总线
		IIC_Init();			
		//复位MPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X80);	
		delay_ms(100);
		//唤醒MPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X00);	 
		//陀螺仪传感器,±2000dps 65536/4000=16.4 LSB
		MPU_Set_Gyro_Fsr(3);				
		//加速度传感器,±8g 65536/16=4096 LSB
		MPU_Set_Accel_Fsr(2);		
		//设置采样率500Hz		
		MPU_Set_Rate(500);						
		//关闭所有中断
		Write_Byte(MPU_INT_EN_REG,0X00);	
		//I2C主模式关闭
		Write_Byte(MPU_USER_CTRL_REG,0X00);	
		//关闭FIFO
		Write_Byte(MPU_FIFO_EN_REG,0X00);	
		//INT引脚低电平有效
		Write_Byte(MPU_INTBP_CFG_REG,0X80);	
		res=Read_Byte(MPU_DEVICE_ID_REG);
		//设置CLKSEL,PLL Z轴为参考
		Write_Byte(MPU_PWR_MGMT1_REG,0X03);	
		//加速度与陀螺仪都工作，不进入低功耗模式
		Write_Byte(MPU_PWR_MGMT2_REG,0X00);					
		return 0;
	}
	/*
	 * 函数功能：设置MPU6050陀螺仪传感器满量程范围 
	 * fsr：0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
	 * 返回值:0,设置成功 其他,设置失败 
	 */
	u8 MPU_Set_Gyro_Fsr(u8 fsr)
	{
		//设置陀螺仪满量程范围  
		return Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
	}
	/*
	 * 函数功能：设置MPU6050加速度传感器满量程范围
	 * fsr：0,±2g;1,±4g;2,±8g;3,±16g
	 * 返回值:0,设置成功 其他,设置失败 
	 */
	u8 MPU_Set_Accel_Fsr(u8 fsr)
	{
		//设置加速度传感器满量程范围  
		return Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
	}
	/*
	 * 函数功能：设置MPU6050的数字低通滤波器
	 * lpf：数字低通滤波频率(Hz)
	 * 返回值:0,设置成功 其他,设置失败 
	 */
	u8 MPU_Set_LPF(u16 lpf)
	{
		u8 data=0;
		if(lpf>=188)data=1;
		else if(lpf>=98)data=2;
		else if(lpf>=42)data=3;
		else if(lpf>=20)data=4;
		else if(lpf>=10)data=5;
		else data=6; 
		//设置数字低通滤波器  
		return Write_Byte(MPU_CFG_REG,data);
	}
	/*
	 * 函数功能：设置MPU6050的采样率(假定Fs=1KHz)
	 * rate：4~1000(Hz)
	 * 返回值:0,设置成功 其他,设置失败 
	 */
	u8 MPU_Set_Rate(u16 rate)
	{
		u8 data;
		if(rate>1000)rate=1000;
		if(rate<4)rate=4;
		data=1000/rate-1;
		data=Write_Byte(MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
		return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
	}

	/*
	 * 函数功能：得到温度值
	 * 返回值:温度值(扩大了10倍)
	 */
	int MPU_Get_Temperature(void)
	{
		float Temp;
	  Temp=(Read_Byte(MPU_TEMP_OUTH_REG)<<8)+Read_Byte(MPU_TEMP_OUTL_REG);
		//数据类型转换
		if(Temp>32768) 
			Temp-=65536;	
		//温度放大十倍存放
		Temp=(36.53+Temp/340)*10;	  
	  return (int)Temp;
	}
	/*
	 * 函数功能：得到陀螺仪值(原始值)
	 * gx,gy,gz：陀螺仪x,y,z轴的原始读数(带符号)
	 * 返回值:0,成功 其他,错误代码 
	 */
	u8 MPU_Get_Gyroscope(short *gx,short *gy,short *gz)
	{
		u8 buf[6],res;  
		res=Read_Len(MPU_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
		if(res==0)
		{
			*gx=((u16)buf[0]<<8)|buf[1];  
			*gy=((u16)buf[2]<<8)|buf[3];  
			*gz=((u16)buf[4]<<8)|buf[5];
		} 	
		return res;;
	}
	/*
	 * 函数功能：得到加速度值(原始值)
	 * gx,gy,gz：陀螺仪x,y,z轴的原始读数(带符号)
	 * 返回值:0,成功 其他,错误代码 
	 */
	u8 MPU_Get_Accelerometer(float *ax,float *ay,float *az)
	{
		u8 buf[6],res;  
		res=Read_Len(MPU_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
		if(res==0)
		{
			*ax = (int16_t)(((uint16_t)buf[0] << 8)  | (uint16_t)buf[1]);
			*ay = (int16_t)(((uint16_t)buf[2] << 8)  | (uint16_t)buf[3]);
			*az = (int16_t)(((uint16_t)buf[4] << 8)  | (uint16_t)buf[5]);
		} 	
		return res;;
	}

	/*
	 * 函数功能：读取MPU6050内置DMP的姿态信息
	 */
	void Read_DMP(void)
	{	
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
	}
	
	/*
	 * 函数功能：读取MPU6050版本信息
	 */
	void Mpu6050FindRevision(void)
	{
			uint8_t readBuffer[6];
			uint8_t ack = !Read_Len(MPU_ADDR, MPU_ACCEL_OFFS_REG, 6, readBuffer);
			uint8_t revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
			if (ack && revision) 
			{
					if (revision == 1) 
					{
							_mpu_half_resolution = 1;
					} 
					else if (revision == 2) 
					{
							_mpu_half_resolution = 0;
					} 
					else if ((revision == 3) || (revision == 7)) 
					{
							_mpu_half_resolution = 0;
					}
			} 
			else 
			{
					uint8_t productId;
					ack = !Read_Len(MPU_ADDR, MPU_PROD_ID_REG, 1, &productId);
					revision = productId & 0x0F;
					if (!ack || revision == 0) 
					{
						
					} 
					else if (revision == 4) 
					{
							_mpu_half_resolution = 1;
					} 
					else 
					{
							_mpu_half_resolution = 0;
					}
			}
	}
	
	/**************************************************
	 * 函数功能：读取MPU6050原始信息
	 * 返回值：读取成功返回1
	 * 说明：函数中进行卡尔曼滤波，得到较稳定的值后再读取数据进行静态偏置计算
	 **************************************************/
	uint8_t MPU6050_read(float* acc, float* gyro)
	{				
		float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Accel_Angle_z,Gyro_X,Gyro_Z,Gyro_Y;
		uint8_t IMUBuffer[14];
		
		if(!Read_Len(MPU_ADDR, 0x3B, 14, IMUBuffer))
		{
			Accel_X = (int16_t)(((uint16_t)IMUBuffer[0] << 8)  | (uint16_t)IMUBuffer[1]);
			Accel_Y = (int16_t)(((uint16_t)IMUBuffer[2] << 8)  | (uint16_t)IMUBuffer[3]);
			Accel_Z = (int16_t)(((uint16_t)IMUBuffer[4] << 8)  | (uint16_t)IMUBuffer[5]);
			Gyro_X  = (int16_t)(((uint16_t)IMUBuffer[8] << 8)  | (uint16_t)IMUBuffer[9]);
			Gyro_Y  = (int16_t)(((uint16_t)IMUBuffer[10] << 8) | (uint16_t)IMUBuffer[11]);
			Gyro_Z  = (int16_t)(((uint16_t)IMUBuffer[12] << 8) | (uint16_t)IMUBuffer[13]);	
			//数据类型转换  
			if(Gyro_X>32768)  Gyro_X-=65536;                
			if(Gyro_Y>32768)  Gyro_Y-=65536;                
			if(Gyro_Z>32768)  Gyro_Z-=65536;                
			if(Accel_X>32768) Accel_X-=65536;               
			if(Accel_Y>32768) Accel_Y-=65536;                
			if(Accel_Z>32768) Accel_Z-=65536;  
			//陀螺仪量程转换
			Gyro_X=Gyro_X/16.4;                              
			Gyro_Y=Gyro_Y/16.4;    
			Gyro_Z=Gyro_Z/16.4;	
			//ACC赋值
			acc[0]=Accel_X;acc[1]=Accel_Y;acc[2]=Accel_Z;
			//GYRO赋值
			gyro[0]=Gyro_X;
			gyro[1]=Gyro_Y;
			gyro[2]=Gyro_Z;
			return 1; 	
		}			
		return 0;
	}
	/*****************************************************
	函数功能：用四元数法解算姿态 一阶龙格库塔
	入口参数：GYRO，ACC 角速度与加速度
	返回值：无
	*****************************************************/
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;//四元数定义与初始化
	float ki=0.001;//积分系数 自行调节
	float halfT=0.0025;//积分周期的一半
	float kp=0.75;//互补滤波系数
	void attitude_algorithm1(float *GYRO,float *ACC,float *pitch,float *roll,float *yaw)
	{
		float norm;//归一化分母
		float vx,vy,vz;//重力分量
		float ex,ey,ez;//姿态误差
		static float accex,accey,accez;
		//陀螺仪转化为弧度
		GYRO[0]*=0.0174f;
		GYRO[1]*=0.0174f;
		GYRO[2]*=0.0174f;
		//加速度归一化
		ACC[0]/=16384;
		ACC[1]/=16384;
		ACC[2]/=16384;
		norm=sqrt(ACC[0]*ACC[0]+ACC[1]*ACC[1]+ACC[2]*ACC[2]);
		ACC[0]/=norm;
		ACC[1]/=norm;
		ACC[2]/=norm;
		//获得重力分量
		vx=2*(q1*q3-q0*q2);
		vy=2*(q0*q1+q2*q3);
		vz=1-2*q1*q1-2*q2*q2;
		//求姿态误差
		ex=ACC[1]*vz-ACC[2]*vy;
		ey=ACC[2]*vx-ACC[0]*vz;
		ez=ACC[0]*vy-ACC[1]*vx;
		//对误差积分
		accex+=ex*ki*2*halfT;
		accey+=ex*ki*2*halfT;
		accez+=ex*ki*2*halfT;
		//互补滤波
		GYRO[0]+=kp*ex+accex;
		GYRO[1]+=kp*ey+accey;
		GYRO[2]+=kp*ez+accez;
		//解四元数微分方程
		q0+=halfT*(-q1*GYRO[0]-q2*GYRO[1]-q3*GYRO[2]);
		q1+=halfT*(q0*GYRO[0]-q3*GYRO[1]+q2*GYRO[2]);
		q2+=halfT*(q0*GYRO[1]-q1*GYRO[2]+q3*GYRO[0]);
		q3+=halfT*(q0*GYRO[2]+q1*GYRO[1]-q2*GYRO[0]);
		//四元数归一化
		norm=sqrt(q1*q1+q0*q0+q2*q2+q3*q3);
		q0/=norm;
		q1/=norm;
		q2/=norm;
		q3/=norm;
		// 计算俯仰角（pitch）
    *pitch = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180 / 3.14159;  
    // 计算横滚角（roll）
    *roll = asinf(2*(q0*q2 - q1*q3)) * 180 / 3.14159; 
    // 计算偏航角（yaw）
    *yaw = atan2f(2*(q1*q2 + q0*q3), 1 - 2*(q2*q2 + q3*q3)) * 180 / 3.14159;
	}
	/*
	 * 函数功能：获取角度未滤波的值 加速度和陀螺仪滤波的值	
	 * Pitch，Roll，Yaw GYRO存储角速度值 ACC存储加速度值
	 */
	void Get_UnfilterAngle(float *pitch,float *roll,float *GYRO,float *ACC)
	{
		float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Accel_Angle_z,Gyro_X,Gyro_Z,Gyro_Y;
		uint8_t IMUBuffer[14];
		if(!Read_Len(MPU_ADDR, 0x3B, 14, IMUBuffer))
			{
					Accel_X = (int16_t)(((uint16_t)IMUBuffer[0] << 8)  | (uint16_t)IMUBuffer[1]);
					Accel_Y = (int16_t)(((uint16_t)IMUBuffer[2] << 8)  | (uint16_t)IMUBuffer[3]);
					Accel_Z = (int16_t)(((uint16_t)IMUBuffer[4] << 8)  | (uint16_t)IMUBuffer[5]);
					Gyro_X  = (int16_t)(((uint16_t)IMUBuffer[8] << 8)  | (uint16_t)IMUBuffer[9]);
					Gyro_Y  = (int16_t)(((uint16_t)IMUBuffer[10] << 8) | (uint16_t)IMUBuffer[11]);
					Gyro_Z  = (int16_t)(((uint16_t)IMUBuffer[12] << 8) | (uint16_t)IMUBuffer[13]);	
			}
			//数据类型转换  
			if(Gyro_X>32768)  Gyro_X-=65536;                
			if(Gyro_Y>32768)  Gyro_Y-=65536;                
			if(Gyro_Z>32768)  Gyro_Z-=65536;                
			if(Accel_X>32768) Accel_X-=65536;               
			if(Accel_Y>32768) Accel_Y-=65536;                
			if(Accel_Z>32768) Accel_Z-=65536;   		
			//陀螺仪量程转换
			Gyro_X=Gyro_X/16.4;                              
			Gyro_Y=Gyro_Y/16.4;    
			Gyro_Z=Gyro_Z/16.4;	
			Gyro_X-=gyro_offset[0];
			Gyro_Y-=gyro_offset[0];
			Gyro_Z-=gyro_offset[0];
			//卡尔曼滤波
			Gyro_X=KalmanFilter1(&k1,Gyro_X);
			Gyro_Y=KalmanFilter1(&k2,Gyro_Y);
			Gyro_Z=KalmanFilter1(&k3,Gyro_Z);
			Accel_X=KalmanFilter1(&k4,Accel_X);
			Accel_Y=KalmanFilter1(&k5,Accel_Y);
			Accel_Z=KalmanFilter1(&k6,Accel_Z);
	  	//计算倾角，转换单位为度
			Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/3.14159;     	
			Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/3.14159; 
			//陀螺仪
			GYRO[0]=Gyro_X;
			GYRO[1]=Gyro_Y;
			GYRO[2]=Gyro_Z;
			//加速度
			ACC[0]=Accel_X;ACC[1]=Accel_Y;ACC[2]=Accel_Z;
			*pitch=Accel_Angle_x;
			*roll=Accel_Angle_y;
	}
	/*
	 * 函数功能：获取角度
	 * way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
	 */
	void Get_Angle(int way)
	{ 
		float Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
		uint8_t IMUBuffer[14];
		if(way==2)
		{
			mpu_dmp_get_data(&Roll,&Pitch,&Yaw);
			if(!Read_Len(MPU_ADDR, 0x3B, 14, IMUBuffer))
			{
					Accel_X = (int16_t)(((uint16_t)IMUBuffer[0] << 8)  | (uint16_t)IMUBuffer[1]);
					Accel_Y = (int16_t)(((uint16_t)IMUBuffer[2] << 8)  | (uint16_t)IMUBuffer[3]);
					Accel_Z = (int16_t)(((uint16_t)IMUBuffer[4] << 8)  | (uint16_t)IMUBuffer[5]);
					Gyro_X  = (int16_t)(((uint16_t)IMUBuffer[8] << 8)  | (uint16_t)IMUBuffer[9]);
					Gyro_Y  = (int16_t)(((uint16_t)IMUBuffer[10] << 8) | (uint16_t)IMUBuffer[11]);
					Gyro_Z  = (int16_t)(((uint16_t)IMUBuffer[12] << 8) | (uint16_t)IMUBuffer[13]);	
			}
				//数据类型转换  
				if(Gyro_X>32768)  Gyro_X-=65536;                
				if(Gyro_Y>32768)  Gyro_Y-=65536;                
				if(Gyro_Z>32768)  Gyro_Z-=65536;                
				if(Accel_X>32768) Accel_X-=65536;               
				if(Accel_Y>32768) Accel_Y-=65536;                
				if(Accel_Z>32768) Accel_Z-=65536;   		
				//陀螺仪量程转换
				Gyro_X=Gyro_X/16.4;                              
				Gyro_Y=Gyro_Y/16.4;    
				Gyro_Z=Gyro_Z/16.4;	
				//静态偏置补偿
				Gyro_X-=gyro_offset[0];
				Gyro_Y-=gyro_offset[1];
				Gyro_Z-=gyro_offset[2];
				Gyro_Balance=Gyro_X;
			//计算倾角，转换单位为度
				Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     	
				Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI;  
			//卡尔曼滤波
				Pitch = Kalman_Filter_x(Accel_Angle_x,Gyro_X);
				Roll  = Kalman_Filter_y(Accel_Angle_y,Gyro_Y);
				
				Angle_Balance=Pitch;                              //更新平衡倾角
				Gyro_Turn=Gyro_Z;                                 //更新转向角速度
				Acceleration_Z=Accel_Z;                           //更新Z轴加速度计
		}
		else if(way==1)
		{
			if(mpu_dmp_get_data(&Roll,&Pitch,&Yaw)==0)
			{ 
				if(!Read_Len(MPU_ADDR, 0x3B, 14, IMUBuffer))
			{
					Accel_X = (int16_t)(((uint16_t)IMUBuffer[0] << 8)  | (uint16_t)IMUBuffer[1]);
					Accel_Y = (int16_t)(((uint16_t)IMUBuffer[2] << 8)  | (uint16_t)IMUBuffer[3]);
					Accel_Z = (int16_t)(((uint16_t)IMUBuffer[4] << 8)  | (uint16_t)IMUBuffer[5]);
					Gyro_X  = (int16_t)(((uint16_t)IMUBuffer[8] << 8)  | (uint16_t)IMUBuffer[9]);
					Gyro_Y  = (int16_t)(((uint16_t)IMUBuffer[10] << 8) | (uint16_t)IMUBuffer[11]);
					Gyro_Z  = (int16_t)(((uint16_t)IMUBuffer[12] << 8) | (uint16_t)IMUBuffer[13]);	
			}
				//数据类型转换  
				if(Gyro_X>32768)  Gyro_X-=65536;                
				if(Gyro_Y>32768)  Gyro_Y-=65536;                
				if(Gyro_Z>32768)  Gyro_Z-=65536;                
				if(Accel_X>32768) Accel_X-=65536;               
				if(Accel_Y>32768) Accel_Y-=65536;                
				if(Accel_Z>32768) Accel_Z-=65536;   		
				//陀螺仪量程转换
				gyro[0]=Gyro_X/16.4;                              
				gyro[1]=Gyro_Y/16.4;    
				gyro[2]=Gyro_Z/16.4;	
				//静态偏置补偿
				gyro[0]-=gyro_offset[0];
				gyro[1]-=gyro_offset[1];
				gyro[2]-=gyro_offset[2];
			  Pitch = Kalman_Filter_x(Pitch,gyro[0]);
				Roll  = Kalman_Filter_y(Roll,gyro[1]);
				Angle_Balance=Pitch;             	 				//更新平衡倾角,前倾为正，后倾为负
				Gyro_Balance=gyro[0];              			  //更新平衡角速度,前倾为正，后倾为负
				Gyro_Turn=gyro[2];                 				//更新转向角速度
				Acceleration_Z=accel[2];           				//更新Z轴加速度计
			}
		}
}
/*MPU6050操作*/
