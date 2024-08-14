/**
  *****************************************************************************
  * @file			key.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/04/25
	* @environment	stm32f407
  * @brief   
  *****************************************************************************
**/




#ifndef __KEY_H
#define __KEY_H
#include "sys.h"




typedef struct
{
	u8 number; /* 按键序号 */
	
	GPIO_TypeDef* GPIOx;
	u16 Pin;
	GPIOPuPd_TypeDef PuPd;
	u8 KEY_PRESS_DOWN_LEVEL;
	
	float filter_time; /* 滤波时间 */
	float filter_count; /*滤波次数计数*/
	
	float long_press_time; /* 长按所需时间 0表示不检测长按*/
	float long_press_count;
	
	float repeat_time; /* 长按效果间隔时间 0表示支持连按*/
	float repeat_count;
	
	u8 state; /* 按键状态 0:未按下 1:按下 */
	
}_key;




void key_config(_key *K);
void key_Init(void);
void key_check(_key *K,float T);
void key_scan(float T);

void key_dowm_fun(u8 key_num);

/* 可修改部分 */
//========================
void key1_function(void);
void key2_function(void);
void key3_function(void);
//========================




#endif

/* end of key.h */
