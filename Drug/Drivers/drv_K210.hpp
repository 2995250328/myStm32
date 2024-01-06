#pragma once

#include <stdint.h>
#include "Basic.hpp"

#ifdef __cplusplus
	extern "C" {
#endif
		
/*
 * 函数功能：串口初始化
 * 入口参数：波特率
 * 返回  值：无
 */
void init_drv_Uart4(u32 bound);
void K210_Check_Data_Task(void);
void K210_Transmit(u8* Tx);
		
		
#ifdef __cplusplus
	}
#endif
