#ifndef __GW_GRAY_SERIAL_H__
#define __GW_GRAY_SERIAL_H__

#include <stdint.h>
#include <stm32f10x_gpio.h>

uint8_t gw_gray_serial_read(GPIO_TypeDef *GPIOx, uint16_t gpio_clk, uint16_t gpio_dat);


#endif //__GW_GRAY_SERIAL_H__
