#pragma once

#include <stdbool.h>
#include <stdio.h>
#include "sys.h"
#include "stm32f10x.h"

#define KEY GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_0)   
//�Ƿ�ʹ��Ӳ��ȥ��
#define Is_Detrembling 0
void EXTIX_Init(void);
void init_drv_Key(void);       //������ʼ��
u8 click_N_Double (u16 time);  //��������ɨ���˫������ɨ��
u8 click(void);               //��������ɨ��
u8 Long_Press(void);          //�������
u8  select(void);             //ѡ�����е�ģʽ
void Key(void);				  //�����޸�ģʽ
