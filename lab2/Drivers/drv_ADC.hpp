#pragma once
#include "Basic.hpp"

#define Battery_Ch 12
/*
 * 函数功能：AD采样
 * 入口参数: ch：ADC1 的通道
 * 返回  值：AD转换结果
 */	 		
u16 Get_Adc(u8 ch);
/*
 * 函数功能：读取电池电压 
 * 入口参数: way 获取电压的算法 1：无  2：卡尔曼 3：互补滤波 4：低通
 * 返回  值：电池电压 单位MV
 */
float Get_battery_volt(int way);
//ADC初始化函数
void init_drv_ADC(void);