#include "gw_gray_serial.h"

#define GW_GRAY_DELAY_TICK 27

/* 8MHz 下5us大概是27 */
static void gw_gray_delay(uint32_t delay)
{
	volatile uint32_t delay_tick = delay;
	
	while (delay_tick > 0) {
		delay_tick--;
	}
}

uint8_t gw_gray_serial_read(GPIO_TypeDef *GPIOx, uint16_t gpio_clk, uint16_t gpio_dat)
{
	uint8_t ret = 0;
	uint8_t i;

	for (i = 0; i < 8; ++i) {
		/* 输出时钟下降沿 */
		GPIO_ResetBits(GPIOx, gpio_clk);
		gw_gray_delay(GW_GRAY_DELAY_TICK); //有外部上拉源(4k ~ 10k电阻) 可不加此行

		ret |= GPIO_ReadInputDataBit(GPIOx, gpio_dat) << i;

		/* 输出时钟上升沿,让传感器更新数据*/
		GPIO_SetBits(GPIOx, gpio_clk);

		/* 延迟需要在5us左右 */
		gw_gray_delay(GW_GRAY_DELAY_TICK);
	}
	
	return ret;
}

