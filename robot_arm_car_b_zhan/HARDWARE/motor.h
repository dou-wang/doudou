/**
  *****************************************************************************
  * @file			motor.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/






#ifndef __MOTO_H
#define __MOTO_H
#include "sys.h"






void TIM1_PWM_Init(u32 arr,u32 psc);

#define MOTOR_ARR	2200 /* 10kHz */
#define MOTOR_PRE	2				
 



#define MOTOR1_SPEED	TIM8->CCR1
#define MOTOR2_SPEED	TIM8->CCR2
#define MOTOR3_SPEED	TIM8->CCR3
#define MOTOR4_SPEED	TIM8->CCR4




/* 自定义修改 */
/////////////////////////////////////////////////
#define MOTOR12_ALL_ON		PAout(11)=1
#define MOTOR12_ALL_OFF		PAout(11)=0

#define MOTOR1_OFF			PGout(2)=1;PGout(3)=1	
#define MOTOR1_R			PGout(2)=0;PGout(3)=1   /* 顺时针 */
#define MOTOR1_L			PGout(2)=1;PGout(3)=0 
		
#define MOTOR2_OFF			PGout(4)=1;PGout(5)=1	
#define MOTOR2_R			PGout(4)=1;PGout(5)=0 /* 顺时针 */
#define MOTOR2_L			PGout(4)=0;PGout(5)=1	

/////////////////////////////////////////////////
#define MOTOR34_ALL_ON		PAout(12)=1
#define MOTOR34_ALL_OFF		PAout(12)=0

#define MOTOR3_OFF			PGout(6)=1;PGout(8)=1	
#define MOTOR3_R			PGout(6)=0;PGout(8)=1 /* 顺时针 */
#define MOTOR3_L			PGout(6)=1;PGout(8)=0
		
#define MOTOR4_OFF			PDout(14)=1;PDout(15)=1	
#define MOTOR4_R			PDout(14)=1;PDout(15)=0	 /* 顺时针 */
#define MOTOR4_L			PDout(14)=0;PDout(15)=1	
/////////////////////////////////////////////////




void Encoder_Init_TIM2(void);

void Encoder_Init_TIM5(void);
void motor_Init(u16 arr,u16 pre);
void motor_state_Init(void);
void motor_on(void);
void motor_off(void);
void Encoder_Init_TIM3(void);

void Encoder_Init_TIM4(void);



#endif



/*end of motor.h*/


