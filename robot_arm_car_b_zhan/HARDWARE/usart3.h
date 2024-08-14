/**
  *****************************************************************************
  * @file				usart3.h
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/7
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#ifndef __USART3_H
#define __USART3_H
#include "sys.h"
#include "string.h"
#include "stdio.h"

void usart3_Init(u32 B);
void usart3_send(char *data, u8 num);
void usart3_send1(char *data, u8 num, u8 x);
void Calibration(void);
void Zero_Clearing(void);
void Update_A(float angle);
void Update_X(float posx);
void Update_Y(float posy);
void Update_XY(float posx,float posy);

extern char txBuffer[];
extern float pos_x;
extern float pos_y;
extern float zangle;



#endif




/* end of usart3.h */
