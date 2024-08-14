/**
  *****************************************************************************
  * @file				uart4.h
  * @author				WWJ
  * @version			v1.0
  * @date				2018/10/25
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#ifndef __UART4_H
#define __UART4_H
#include "sys.h"



typedef struct
{
	u8 finish; /* Îª0Ê±¿ªÆôÉ¨Ãè */
	
	u8 first_num;
	u8 second_num;
	u8 third_num;
	int num;
	
}_CODE;



void uart4_Init(u32 B);
void uart4_send(u8 *data, u8 num);



extern _CODE Code;



#endif




/* end of uart4.h */
