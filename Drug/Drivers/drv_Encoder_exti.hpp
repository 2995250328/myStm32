#pragma once
#include "sys.h"
#include "delay.h"

//右轮外部中断编码器初始化
void Encoder_Init_Exit_Right(void);
//读取右轮编码器数值
int Encoder_Readnum_Right(void);
//左轮外部中断编码器初始化
void Encoder_Init_Exit_Left(void);
//读取左轮编码器数值
int Encoder_Readnum_left(void);
