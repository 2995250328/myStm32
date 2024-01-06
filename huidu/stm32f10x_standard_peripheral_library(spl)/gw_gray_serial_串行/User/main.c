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
#include <stm32f10x_gpio.h>
#include "gw_grayscale_sensor.h"
#include "gw_gray_serial.h"

#define GW_GRAY_SERIAL_GPIO_GROUP GPIOB
#define GW_GRAY_SERIAL_GPIO_CLK GPIO_Pin_8
#define GW_GRAY_SERIAL_GPIO_DAT GPIO_Pin_9

void delay() {
	volatile unsigned int delay_count = 10000;
	while (delay_count > 0) {
		delay_count --;
	}
}

/**
 * 初始化gpio
 */
void spl_gw_gray_serial_init()
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GW_GRAY_SERIAL_GPIO_DAT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GW_GRAY_SERIAL_GPIO_GROUP, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GW_GRAY_SERIAL_GPIO_CLK;
	GPIO_Init(GW_GRAY_SERIAL_GPIO_GROUP, &GPIO_InitStructure);
}

int main()
{
	uint8_t data_arr[8] = {0};
	
	// 此处的"volatile"仅用于debug观察数据（volatile会阻止编译器对目标的优化），移植时请去掉"volatile"修饰词
	volatile uint8_t sensor_data = 0;

	spl_gw_gray_serial_init();

	/* 读取数据测试 */
	while(1)
	{
		// 读取开关量数据
		sensor_data = gw_gray_serial_read(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, GW_GRAY_SERIAL_GPIO_DAT);
		
				/*
		* 如何使用一个字节有八个开关量的数据
		*/
		if (sensor_data == 0xF0) { // = 0b11110000
			/* 测试8~5探头为1, 4~1探头为0 */
		}
		
		/* 从字节中提取单个比特 */
		if(GET_NTH_BIT(sensor_data, 1)) {
			// 探头1亮了
		}
		
		if(GET_NTH_BIT(sensor_data, 4)) {
			// 探头4亮了
		}
		
		/* 分散字节里的比特到八个变量里这里为data_arr[0] ~ data_arr[7], 也可以是val1 ~ val8, 因为是宏定义 */
		SEP_ALL_BIT8(sensor_data, 
			data_arr[0], // 探头1
			data_arr[1], // 探头2
			data_arr[2], // 探头3
			data_arr[3], // 探头4
			data_arr[4], // 探头5
			data_arr[5], // 探头6
			data_arr[6], // 探头7
			data_arr[7]  // 探头8
		);
		
		if(data_arr[0]) {
			// 探头1亮了
		}

		delay();
	}
}
