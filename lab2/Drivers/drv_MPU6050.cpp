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
��ע��IIC_Send_Byte((MPU_ADDR<<1)|0);������������һλ��Ϊ������һλ����λ 1�� 0д
	Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);������λ����Ϊ�üĴ�������λû������ Ҫ������������λ���ܴ浽ָ��λ
	*gx=((u16)buf[0]<<8)|buf[1];  
	*gy=((u16)buf[2]<<8)|buf[3];  
	*gz=((u16)buf[4]<<8)|buf[5];�߰�λ��Ͱ�λ
*/
using namespace std;

//�����Ǿ�̬ƫ����
float gyro_offset[3]={-4.0105,0.7712,-0.7924};
//���ٶ�У׼
float acc_offset[3]={613.9229,-256.7191,561.9164};
//�����Ǳ���
short gyro[3], accel[3], sensors;
//���ٶ�ֵ
float GYRO[3]={0};
//���ٶ�ֵ
float ACC[3]={0};
//����Ƕ� Z��������
float Pitch,Roll,Yaw,Gryo_Z; 
//���ٶȴ�����ԭʼ����
float aacx,aacy,aacz;	
//������ԭʼ����
short gyrox,gyroy,gyroz;	

int16_t accRaw[3], gyroRaw[3];
int16_t temperuteRaw;
static uint8_t _mpu_half_resolution;
extern pt1Filter_t filter,f1,f2,f3,f4,f5,f6;
extern Kalman kfp,k1,k2,k3,k4,k5,k6;

/* IIC���� */
	
	/*
	 * �������ܣ�IICдһ���ֽ� 
	 * reg���Ĵ�����ַ
	 * data������
	 * ����ֵ:0,���� ����,�������
	 */
	u8 Write_Byte(u8 reg,u8 data) 				 
	{ 
	
		IIC_Start(); 
		IIC_Delay();
		//����������ַ+д����	
		IIC_Send_Byte((MPU_ADDR<<1)|0);
		//�ȴ�Ӧ��
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();		 
			return 1;		
		}
		//д�Ĵ�����ַ
		IIC_Send_Byte(reg);	
		//�ȴ�Ӧ�� 
		IIC_Wait_Ack();		
		//��������
		IIC_Send_Byte(data);
		//�ȴ�ACK
		if(IIC_Wait_Ack())	
		{
			IIC_Stop();	 
			return 1;		 
		}		 
		IIC_Stop();	 
		return 0;
	}		
	/*
	 * �������ܣ�IIC��һ���ֽ� 
	 * reg���Ĵ�����ַ
	 * ����ֵ:����������
	 */
	u8 Read_Byte(u8 reg)
	{
		u8 res;
		IIC_Start(); 
		//����������ַ+д����	
		IIC_Send_Byte((MPU_ADDR<<1)|0);
		//�ȴ�Ӧ�� 
		IIC_Wait_Ack();		
		//д�Ĵ�����ַ
		IIC_Send_Byte(reg);	
		//�ȴ�Ӧ��
		IIC_Wait_Ack();		
		IIC_Start();
		//����������ַ+������	
		IIC_Send_Byte((MPU_ADDR<<1)|1);
		//�ȴ�Ӧ��
		IIC_Wait_Ack();		 
		//��ȡ����,����nACK 
		res=IIC_Read_Byte(0);
		//����һ��ֹͣ���� 
		IIC_Stop();			
		return res;		
	}

/* IIC���� */

/* MPU6050���� */
	
	/*
	 * �������ܣ���ʼ��MPU6050
	 * ����ֵ:0,�ɹ� ����,�������
	 */
	u8 MPU_Init(void)
	{ 
		u8 res;
		//��ʼ��IIC����
		IIC_Init();			
		//��λMPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X80);	
		delay_ms(100);
		//����MPU6050
		Write_Byte(MPU_PWR_MGMT1_REG,0X00);	 
		//�����Ǵ�����,��2000dps 65536/4000=16.4 LSB
		MPU_Set_Gyro_Fsr(3);				
		//���ٶȴ�����,��8g 65536/16=4096 LSB
		MPU_Set_Accel_Fsr(2);		
		//���ò�����500Hz		
		MPU_Set_Rate(500);						
		//�ر������ж�
		Write_Byte(MPU_INT_EN_REG,0X00);	
		//I2C��ģʽ�ر�
		Write_Byte(MPU_USER_CTRL_REG,0X00);	
		//�ر�FIFO
		Write_Byte(MPU_FIFO_EN_REG,0X00);	
		//INT���ŵ͵�ƽ��Ч
		Write_Byte(MPU_INTBP_CFG_REG,0X80);	
		res=Read_Byte(MPU_DEVICE_ID_REG);
		//����CLKSEL,PLL Z��Ϊ�ο�
		Write_Byte(MPU_PWR_MGMT1_REG,0X03);	
		//���ٶ��������Ƕ�������������͹���ģʽ
		Write_Byte(MPU_PWR_MGMT2_REG,0X00);					
		return 0;
	}
	/*
	 * �������ܣ�����MPU6050�����Ǵ����������̷�Χ 
	 * fsr��0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
	 * ����ֵ:0,���óɹ� ����,����ʧ�� 
	 */
	u8 MPU_Set_Gyro_Fsr(u8 fsr)
	{
		//���������������̷�Χ  
		return Write_Byte(MPU_GYRO_CFG_REG,fsr<<3);
	}
	/*
	 * �������ܣ�����MPU6050���ٶȴ����������̷�Χ
	 * fsr��0,��2g;1,��4g;2,��8g;3,��16g
	 * ����ֵ:0,���óɹ� ����,����ʧ�� 
	 */
	u8 MPU_Set_Accel_Fsr(u8 fsr)
	{
		//���ü��ٶȴ����������̷�Χ  
		return Write_Byte(MPU_ACCEL_CFG_REG,fsr<<3);
	}
	/*
	 * �������ܣ�����MPU6050�����ֵ�ͨ�˲���
	 * lpf�����ֵ�ͨ�˲�Ƶ��(Hz)
	 * ����ֵ:0,���óɹ� ����,����ʧ�� 
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
		//�������ֵ�ͨ�˲���  
		return Write_Byte(MPU_CFG_REG,data);
	}
	/*
	 * �������ܣ�����MPU6050�Ĳ�����(�ٶ�Fs=1KHz)
	 * rate��4~1000(Hz)
	 * ����ֵ:0,���óɹ� ����,����ʧ�� 
	 */
	u8 MPU_Set_Rate(u16 rate)
	{
		u8 data;
		if(rate>1000)rate=1000;
		if(rate<4)rate=4;
		data=1000/rate-1;
		data=Write_Byte(MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
		return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
	}

	/*
	 * �������ܣ��õ��¶�ֵ
	 * ����ֵ:�¶�ֵ(������10��)
	 */
	int MPU_Get_Temperature(void)
	{
		float Temp;
	  Temp=(Read_Byte(MPU_TEMP_OUTH_REG)<<8)+Read_Byte(MPU_TEMP_OUTL_REG);
		//��������ת��
		if(Temp>32768) 
			Temp-=65536;	
		//�¶ȷŴ�ʮ�����
		Temp=(36.53+Temp/340)*10;	  
	  return (int)Temp;
	}
	/*
	 * �������ܣ��õ�������ֵ(ԭʼֵ)
	 * gx,gy,gz��������x,y,z���ԭʼ����(������)
	 * ����ֵ:0,�ɹ� ����,������� 
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
	 * �������ܣ��õ����ٶ�ֵ(ԭʼֵ)
	 * gx,gy,gz��������x,y,z���ԭʼ����(������)
	 * ����ֵ:0,�ɹ� ����,������� 
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
	 * �������ܣ���ȡMPU6050����DMP����̬��Ϣ
	 */
	void Read_DMP(void)
	{	
		mpu_dmp_get_data(&Pitch,&Roll,&Yaw);
	}
	
	/*
	 * �������ܣ���ȡMPU6050�汾��Ϣ
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
	
	/*
	 * �������ܣ���ȡMPU6050ԭʼ��Ϣ
	 * ����ֵ����ȡ�ɹ�����1
	 */
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

			//��������ת��  
			if(Gyro_X>32768)  Gyro_X-=65536;                
			if(Gyro_Y>32768)  Gyro_Y-=65536;                
			if(Gyro_Z>32768)  Gyro_Z-=65536;                
			if(Accel_X>32768) Accel_X-=65536;               
			if(Accel_Y>32768) Accel_Y-=65536;                
			if(Accel_Z>32768) Accel_Z-=65536;  
			//ACC��ֵ
			acc[0]=Accel_X;acc[1]=Accel_Y;acc[2]=Accel_Z;		
			//GYRO��ֵ
			Gyro_X=Gyro_X/16.4;                              
			Gyro_Y=Gyro_Y/16.4;    
			Gyro_Z=Gyro_Z/16.4;					
			gyro[0]=Gyro_X;
			gyro[1]=Gyro_Y;
			gyro[2]=Gyro_Z;
			return 1; 	
		}			
		return 0;
	}
	/*****************************************************
	�������ܣ�����Ԫ����������̬ һ���������
	��ڲ�����GYRO��ACC ���ٶ�����ٶ�
	����ֵ����
	*****************************************************/
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;//��Ԫ���������ʼ��
	float ki=0.001;//����ϵ�� ���е���
	float halfT=0.0025;//�������ڵ�һ��
	float kp=0.75;//�����˲�ϵ��
	void attitude_algorithm1(float *GYRO,float *ACC,float *pitch,float *roll,float *yaw)
	{
		float norm;//��һ����ĸ
		float vx,vy,vz;//��������
		float ex,ey,ez;//��̬���
		static float accex,accey,accez;
		//������ת��Ϊ����
		GYRO[0]*=0.0174f;
		GYRO[1]*=0.0174f;
		GYRO[2]*=0.0174f;
		//���ٶȹ�һ��
		ACC[0]/=16384;
		ACC[1]/=16384;
		ACC[2]/=16384;
		norm=sqrt(ACC[0]*ACC[0]+ACC[1]*ACC[1]+ACC[2]*ACC[2]);
		ACC[0]/=norm;
		ACC[1]/=norm;
		ACC[2]/=norm;
		//�����������
		vx=2*(q1*q3-q0*q2);
		vy=2*(q0*q1+q2*q3);
		vz=1-2*q1*q1-2*q2*q2;
		//����̬���   ʵ������ƫ��
		ex=ACC[1]*vz-ACC[2]*vy;
		ey=ACC[2]*vx-ACC[0]*vz;
		ez=ACC[0]*vy-ACC[1]*vx;
		//��������
		accex+=ex*ki*2*halfT;
		accey+=ex*ki*2*halfT;
		accez+=ex*ki*2*halfT;
		//�����˲�
		GYRO[0]+=kp*ex+accex;
		GYRO[1]+=kp*ey+accey;
		GYRO[2]+=kp*ez+accez;
		//����Ԫ��΢�ַ���
		q0+=halfT*(-q1*GYRO[0]-q2*GYRO[1]-q3*GYRO[2]);
		q1+=halfT*(q0*GYRO[0]-q3*GYRO[1]+q2*GYRO[2]);
		q2+=halfT*(q0*GYRO[1]-q1*GYRO[2]+q3*GYRO[0]);
		q3+=halfT*(q0*GYRO[2]+q1*GYRO[1]-q2*GYRO[0]);
		//��Ԫ����һ��
		norm=sqrt(q1*q1+q0*q0+q2*q2+q3*q3);
		q0/=norm;
		q1/=norm;
		q2/=norm;
		q3/=norm;
		// ���㸩���ǣ�pitch��
    *pitch = atan2f(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)) * 180 / PI;  
    // �������ǣ�roll��
    *roll = asinf(2*(q0*q2 - q1*q3)) * 180 / PI; 
    // ����ƫ���ǣ�yaw��
    *yaw = atan2f(2*(q1*q2 + q0*q3), 1 - 2*(q2*q2 + q3*q3)) * 180 / PI;
	}
	/*
	 * �������ܣ���ȡ�Ƕ�δ�˲���ֵ ���ٶȺ��������˲���ֵ	
	 * Pitch��Roll��Yaw way�˲���ʽ GYRO�洢���ٶ�ֵ ACC�洢���ٶ�ֵ
	 */
	void Get_UnfilterAngle(int way,float *pitch,float *roll,float *GYRO,float *ACC)
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
			//��������ת��  
			if(Gyro_X>32768)  Gyro_X-=65536;                
			if(Gyro_Y>32768)  Gyro_Y-=65536;                
			if(Gyro_Z>32768)  Gyro_Z-=65536;                
			if(Accel_X>32768) Accel_X-=65536;               
			if(Accel_Y>32768) Accel_Y-=65536;                
			if(Accel_Z>32768) Accel_Z-=65536;   		
			//����������ת��
			Gyro_X=Gyro_X/16.4;                              
			Gyro_Y=Gyro_Y/16.4;    
			Gyro_Z=Gyro_Z/16.4;	
			Gyro_X-=gyro_offset[0];
			Gyro_Y-=gyro_offset[0];
			Gyro_Z-=gyro_offset[0];
			//�˲�
			if(way==1)
			{}
			else if(way==2)//�������˲�
			{

			}
			else if(way==3)//�����˲�
			{}
			else if(way==4)//��ͨ�˲�
			{
				Gyro_X=pt1FilterApply4(&f1,Gyro_X,3,0.005);
				Gyro_Y=pt1FilterApply4(&f2,Gyro_Y,5,0.005);
				Gyro_Z=pt1FilterApply4(&f3,Gyro_Z,5,0.005);
				Accel_X=pt1FilterApply4(&f4,Accel_X,5,0.005);
				Accel_Y=pt1FilterApply4(&f5,Accel_Y,5,0.005);
				Accel_Z=pt1FilterApply4(&f6,Accel_Z,5,0.005);				
			}
	  	//������ǣ�ת����λΪ��
			Accel_Angle_x=atan2(Accel_Y,Accel_Z)*180/PI;     	
			Accel_Angle_y=atan2(Accel_X,Accel_Z)*180/PI; 
			//������
			GYRO[0]=Gyro_X;
			GYRO[1]=Gyro_Y;
			GYRO[2]=Gyro_Z;
			//���ٶ�
			ACC[0]=Accel_X;ACC[1]=Accel_Y;ACC[2]=Accel_Z;
			*pitch=Accel_Angle_x;
			*roll=Accel_Angle_y;
	}
	/*
	 * �������ܣ���ȡ�Ƕ�
	 * way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
	 */
	void Get_Angle(int way,float *pitch,float *roll,float *yaw)
	{ 
		
	}
/*MPU6050����*/
