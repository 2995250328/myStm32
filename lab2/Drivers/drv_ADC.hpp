#pragma once
#include "Basic.hpp"

#define Battery_Ch 12
/*
 * �������ܣ�AD����
 * ��ڲ���: ch��ADC1 ��ͨ��
 * ����  ֵ��ADת�����
 */	 		
u16 Get_Adc(u8 ch);
/*
 * �������ܣ���ȡ��ص�ѹ 
 * ��ڲ���: way ��ȡ��ѹ���㷨 1����  2�������� 3�������˲� 4����ͨ
 * ����  ֵ����ص�ѹ ��λMV
 */
float Get_battery_volt(int way);
//ADC��ʼ������
void init_drv_ADC(void);