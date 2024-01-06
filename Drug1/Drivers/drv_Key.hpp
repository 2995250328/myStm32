#pragma once

#include <stdbool.h>
#include <stdio.h>
#include "sys.h"
#include "stm32f10x.h"

#define KEY GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)   
//是否使用硬件去抖
#define Is_Detrembling 0
void EXTIX_Init(void);
void init_drv_Key(void);       //按键初始化
u8 click_N_Double (u16 time);  //单击按键扫描和双击按键扫描
u8 click(void);               //单击按键扫描
u8 Long_Press(void);          //长按检测
u8  select(void);             //选择运行的模式
void Key(void);				  //按键修改模式
