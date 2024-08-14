/**
  *****************************************************************************
  * @file			systick.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief   
  *****************************************************************************
**/




#ifndef __SYSTICK_H
#define	__SYSTICK_H
#include "sys.h"




void systick_Init(void);
u32 getsystick_us(void);
void delay_us(u32 us);
void delay_ms(u32 ms);
u32 Systick_Get_MS(void);




#endif



/* end of systick.h */

