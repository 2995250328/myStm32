#pragma once

#include <stdint.h>
#include "Basic.hpp"

#ifdef __cplusplus
	extern "C" {
#endif
		
/*
 * �������ܣ����ڳ�ʼ��
 * ��ڲ�����������
 * ����  ֵ����
 */
void init_drv_Uart1(u32 bound);
void OpenMv_Check_Data_Task(void);
		
		
#ifdef __cplusplus
	}
#endif

