/**
  *****************************************************************************
  * @file			systick.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/24
	* @environment	stm32f407
  * @brief   
  * @copyright		HUNAU
  *****************************************************************************
**/



#include "systick.h"


u32 systick_cnt_ms=0; /* Count */


void systick_Init(void)
{
	SysTick_Config(168000000/8000); /* Set Reload value */
	
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); /* AHB clock divided by 8 selected as SysTick clock source */
}


u32 getsystick_us(void)
{
	return (systick_cnt_ms*1000 + (SysTick->LOAD-SysTick->VAL)/SysTick->LOAD*1000);
}


void delay_us(u32 us)
{
	u32 now=getsystick_us();
	
	while(getsystick_us()-now < us);
}


void delay_ms(u32 ms)
{
	while(ms--)
	{
		delay_us(1000);
	}
}


void SysTick_Handler(void) /* 1ms interrupt */
{
	systick_cnt_ms++;
}


u32 Systick_Get_MS(void)
{
	return systick_cnt_ms;
}




/* end of systick.c */
