/**
  *****************************************************************************
  * @file				usart6.h
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/7
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#ifndef __USART6_H
#define __USART6_H
#include "sys.h"




typedef struct
{
	u8 finish; /* Îª0Ê±¿ªÆôÉ¨Ãè */
	
	u8 first_num;
	u8 second_num;
	u8 third_num;
	int num;
	
	short errx;
	short erry;
	
	u8 change;
	
}_OPENMV;



extern _OPENMV Openmv;
extern float t1, t2;


void usart6_Init(u32 B);
void usart6_send(u8 *data, u8 num);



#endif


/* end of usart6.h */
