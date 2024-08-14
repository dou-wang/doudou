/**
  *****************************************************************************
  * @file			dj.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/5/15
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/





#ifndef __DJ_H
#define __DJ_H
#include "sys.h"




#define TIM1_ARR	20000
#define TIM1_PRE	168

#define TIM4_ARR	20000
#define TIM4_PRE	84

#define DJ4_PLUSE	TIM1->CCR1
#define DJ3_PLUSE	TIM1->CCR3
#define DJ2_PLUSE	TIM1->CCR2
#define DJ1_PLUSE	TIM1->CCR4





typedef struct
{
	u16 D1;
	u16 D2;
	u16 D3;
	u16 D4;
	
}_DJ_POSITION;




void dj_Init(void);
u8 dj_set(_DJ_POSITION *D);



extern _DJ_POSITION P0;
extern _DJ_POSITION P1;

extern _DJ_POSITION P2;
extern _DJ_POSITION P3;
extern _DJ_POSITION P4;
extern _DJ_POSITION P5;
extern _DJ_POSITION P6;
extern _DJ_POSITION P7;
extern _DJ_POSITION P8;
extern _DJ_POSITION P9;

extern _DJ_POSITION P10;
extern _DJ_POSITION P11;
extern _DJ_POSITION P12;
extern _DJ_POSITION P13;
extern _DJ_POSITION P14;
extern _DJ_POSITION P15;
extern _DJ_POSITION P16;
extern _DJ_POSITION P17;
extern _DJ_POSITION P18;

extern _DJ_POSITION P19;




#endif 




/*end of dj.h*/

