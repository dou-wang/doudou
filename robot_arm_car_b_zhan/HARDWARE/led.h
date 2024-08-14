/**
  *****************************************************************************
  * @file				led.h
  * @author				WWJ
  * @version			v1.0
  * @date				2019/4/25
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/




#ifndef __LED_H
#define __LED_H
#include "sys.h"




typedef struct
{
	u8 number; /* LED序号 */
	
	GPIO_TypeDef* GPIOx;
	u16 Pin;
	
}_LED;




/* 可修改部分 */
//===================================================================
enum //LED序号
{
	B,
	R,
	G,
	
	LED_NUM,
};
//====
#define B_CHANGE	PDout(0) = ~PDout(0)
#define R_CHANGE	PDout(1) = ~PDout(1)
#define G_CHANGE	PDout(3) = ~PDout(3)
//====
#define B_ON		GPIO_SetBits(led0.GPIOx,led0.Pin)
#define B_OFF		GPIO_ResetBits(led0.GPIOx,led0.Pin) 
//====
#define R_ON		GPIO_SetBits(led1.GPIOx,led1.Pin)
#define R_OFF		GPIO_ResetBits(led1.GPIOx,led1.Pin) 
//====
#define G_ON		GPIO_SetBits(led2.GPIOx,led2.Pin)
#define G_OFF		GPIO_ResetBits(led2.GPIOx,led2.Pin) 
//====
#define LED_INIT	led_Config(&led0); led_Config(&led1); led_Config(&led2)
//====
extern	_LED led0;
extern	_LED led1;
extern	_LED led2;
//===================================================================




void led_Config(_LED *L);
void led_Init(void);
void led_state_ctrl(void);

u8 led_breath(u8 led_num);
u8 led_flash(u8 led_num,u8 led_brightness,u8 led_flash_time,float led_on_time,float led_off_time,float led_gap_time,float T);

void led_task(float T);

void led_lightmode_check(void);
void led_reset(void);

void TIM6_Init(u16 arr,u16 pre);




#endif




/* end of led.h */

