/**
  *****************************************************************************
  * @file				usart1.h
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/8
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#ifndef __USART1_H
#define __USART1_H
#include "sys.h"


/* 通信参数列表 */
#define PID1_P	XPID.TEMP_P
#define PID1_I	XPID.I
#define PID1_D	XPID.D

#define PID2_P	YPID.TEMP_P
#define PID2_I	YPID.I
#define PID2_D	YPID.D

#define PID3_P	APID.P
#define PID3_I	APID.I
#define PID3_D	APID.D



typedef struct
{
	u8 send_pid; /* 发送PID给上位机 */
	
	u8 dir_up;
	u8 dir_dowm;
	u8 dir_left;
	u8 dir_right;
	
}_RX;



void usart1_Init(u32 B);
void usart1_send(u8 *data, u8 num);
void usart1_Task(void);



#endif




/* end of usart1.h */
