#pragma once

#include <stdbool.h>
#include <stdio.h>

//LED模式
enum LED_Mode{
	error,
	error1,
	error2,
	error3,
	processing,
	normal
};

//设置LED模式
void Set_LED_Mode( LED_Mode mode );

//LED初始化函数
void init_drv_LED(void);

/*
	LED调亮度函数
	R、G、B：亮度百分比（0-100）
*/
void set_LedBrightness( float R , float G , float B );

/*
	蜂鸣器频率调节函数
	freq:蜂鸣器频率
*/
void set_BuzzerFreq( unsigned short freq );

/*
	蜂鸣器鸣响函数
	on:是否鸣响
*/
void set_BuzzerOnOff( bool on );

/*
	LED闪烁函数
*/
static void LedFlash_Task();

//LED闪烁函数
void Led_Flash();
