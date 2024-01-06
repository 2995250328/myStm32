/*
 * Copyright (c) 2022 感为智能科技(济南)
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 */

#include "stm32f10x.h"  
#include <stdio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_i2c.h>
#include <stm32f10x_gpio.h>
#include "spl_i2c.h"
#include "gw_grayscale_sensor.h"

/* 测试模拟数据改成0, 测试开关量数据改成1 */
#define GW_READ_DIGITAL_DATA 1

void delay()
{
	volatile unsigned int delay_count = 10000;
	while (delay_count > 0) {
		delay_count --;
	}
}

/**
 * 初始化i2c
 */
void spl_i2c_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	I2C_InitTypeDef I2C_InitStructure;	
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_ClockSpeed = 100000;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1 = 0x00;
	
	I2C_Init(I2C1, &I2C_InitStructure);
	I2C_Cmd(I2C1, ENABLE);
}

/**
 * i2c地址扫描
 * @param scan_addr 扫描出来的地址存放,数值不为0的为扫描到的地址，扫到的地址会挨个放在数组的最前面
 * @return 返回扫描到的设备数量, 0为无设备发现
 */
uint8_t i2c_scan(uint8_t *scan_addr)
{
	int i;
	uint8_t count = 0;
	uint8_t data;
	int8_t ret;
	
	for (i = 1; i < 127; ++i) {
		ret = spl_i2c_read(I2C1, i << 1, &data, 1);
		if (ret == 0) {
			scan_addr[count] = i;
			++count;
		}
	}
	
	return count;
}

int main()
{
	/* 存放扫描到的地址 */
	uint8_t scan_addr[128] = {0};
	volatile uint8_t count;
	
	/* 初始化IIC */
	spl_i2c_init();
	
	/* 扫描开始 */
	count = i2c_scan(scan_addr);


#if GW_READ_DIGITAL_DATA
	/* 读取开关量数据 */
	uint8_t gray_sensor[8];
	uint8_t digital_data;
	
	/* 打开开关量数据模式 */
	spl_i2c_write_byte(I2C1, 0x4C << 1, GW_GRAY_DIGITAL_MODE);
	
	spl_i2c_read_byte(I2C1, 0x4C << 1, &digital_data); // digital_data 有1~8号探头开关数据
	while(1)
	{

		/* 读取开关量数据 */
		spl_i2c_read_byte(I2C1, 0x4C << 1, &digital_data); // digital_data 有1~8号探头开关数据
		
		/*
		 * 如何使用单字节的开关数据
		 */
		
		if (digital_data == 0xF0) { // = 0b11110000
			/* 测试8~5探头为1, 4~1探头为0 */
		}
		
		// 从字节中提取第二个 比特
		if(GET_NTH_BIT(digital_data, 2)) {
			// 探头2亮了=检测到白场
		}
		
		/* 把字节里的8个开关量存到八个变量里，这里为gray_sensor[0] ~ gray_sensor[7], 
		 * 也可以是变量val1 ~ val8, 因为是宏定义 */
		SEP_ALL_BIT8(digital_data, 
			gray_sensor[0], //探头1
			gray_sensor[1], //探头2
			gray_sensor[2], //探头3
			gray_sensor[3], //探头4
			gray_sensor[4], //探头5
			gray_sensor[5], //探头6
			gray_sensor[6], //探头7
			gray_sensor[7]  //探头8
		);
		
		if(gray_sensor[0]) {
			// 探头1亮了=检测到白场
		}
		
		delay();
	}
#else
	/* 读取模拟量数据 */
	uint8_t analog_data[8]; // analog_data[0:7] 有1~8号探头模拟数据
	
	/* 打开模拟值模式, 并且读取模拟数值, 后面可以直接读取 */
	spl_i2c_mem8_read(I2C1, 0x4C << 1, GW_GRAY_ANALOG_MODE, analog_data, 8);

	/* 直接读取 */
	spl_i2c_read(I2C1, 0x4C << 1, analog_data, 8);
	
	while(1)
	{
		/* 读取模拟量数据 */
		spl_i2c_read(I2C1, 0x4C << 1, analog_data, 8);
		delay();
	}
	
#endif

	
}
