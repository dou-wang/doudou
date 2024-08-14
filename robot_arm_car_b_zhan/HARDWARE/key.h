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
	u8 number; /* ������� */
	
	GPIO_TypeDef* GPIOx;
	u16 Pin;
	GPIOPuPd_TypeDef PuPd;
	u8 KEY_PRESS_DOWN_LEVEL;
	
	float filter_time; /* �˲�ʱ�� */
	float filter_count; /*�˲���������*/
	
	float long_press_time; /* ��������ʱ�� 0��ʾ����ⳤ��*/
	float long_press_count;
	
	float repeat_time; /* ����Ч�����ʱ�� 0��ʾ֧������*/
	float repeat_count;
	
	u8 state; /* ����״̬ 0:δ���� 1:���� */
	
}_key;




void key_config(_key *K);
void key_Init(void);
void key_check(_key *K,float T);
void key_scan(float T);

void key_dowm_fun(u8 key_num);

/* ���޸Ĳ��� */
//========================
void key1_function(void);
void key2_function(void);
void key3_function(void);
//========================




#endif

/* end of key.h */
