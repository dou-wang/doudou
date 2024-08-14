/**
  *****************************************************************************
  * @file			key.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/




#include "key.h"
#include "main.h"




/* ���޸Ĳ��� */
//=====================================================
#define FILTER_TIME	20 /* unit: ms */
#define LONG_PRESS_TIME	1000 /* unit: ms */
#define REPEAT_TIME	100 /* unit: ms �ظ�������ʱ���� */
//====
enum
{
	KEY1=0,
	KEY2,
	
	KEY_ALL,
};
//====
_key key1={KEY1,GPIOB,GPIO_Pin_12,GPIO_PuPd_UP,0};
_key key2={KEY2,GPIOB,GPIO_Pin_13,GPIO_PuPd_UP,0};
//====
#define	KEY_INIT	key_config(&key1);key_config(&key2) 		/* ��Ҫ��ʼ���İ��� */
#define	KEY_SCAN	key_check(&key1,T);key_check(&key2,T)	/* ��Ҫɨ��İ��� */
//====
/* ��������ʱ�Ĳ������� */
void key_dowm_fun(u8 key_num) 
{
	if(key_num == KEY1)
	{
		key1_function();
	}
	if(key_num == KEY2)
	{
		key2_function();
	}
}
//====
/* ��������ʱ����ϸ���� */
void key1_function(void)
{
	dj_set(&P0);
	Zero_Clearing();
	motor_off();
}
void key2_function(void)
{
	FLAG.Task_start = 1;
	motor_on();
}
//=====================================================




/* parameter init */
void key_config(_key *K)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(K->GPIOx == GPIOA)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	if(K->GPIOx == GPIOB)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	if(K->GPIOx == GPIOC)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	if(K->GPIOx == GPIOD)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	if(K->GPIOx == GPIOE)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	if(K->GPIOx == GPIOF)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	if(K->GPIOx == GPIOG)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=K->Pin;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_PuPd=K->PuPd;

	GPIO_Init(K->GPIOx,&GPIO_InitStruct);
	
	K->filter_count = 0;
	K->filter_time  = FILTER_TIME;
	
	K->long_press_count = 0;
	K->long_press_time  = LONG_PRESS_TIME;
	
	K->repeat_count = 0;
	K->repeat_time  = REPEAT_TIME;
	
	K->state = 0;
}




void key_Init(void)
{
	KEY_INIT;
}




/* check up dowm */
void key_check(_key *K,float T)
{
	if(GPIO_ReadInputDataBit(K->GPIOx,K->Pin) == K->KEY_PRESS_DOWN_LEVEL) /* ��ⰴ���Ƿ��� */
	{
		if(K->filter_count < K->filter_time) /* �˲� */
		{
			K->filter_count += T;
		}
		else if(K->filter_count >= K->filter_time)
		{
			if(K->state == 0)
			{
				K->state = 1; /*��������*/
				key_dowm_fun(K->number);
			}
			if(K->long_press_time > 0) /* ��ⳤ�� */
			{
				if(K->long_press_count < K->long_press_time)
				{
					K->long_press_count += T;
				}
				else /* �ﵽ������׼ */
				{
					if(K->repeat_time > 0) /*֧������*/
					{
						if(K->repeat_count < K->repeat_time)
						{
							K->repeat_count += T;
						}
						else /* ����ִ�� */
						{
							K->repeat_count = 0;
							key_dowm_fun(K->number);
						}
					}
				}
			}
		}
	}
	else
	{
		K->state = 0;
		K->filter_count = 0;
		K->long_press_count = 0;
		K->repeat_count = 0;
	}
}




/* key scan */
void key_scan(float T)
{
	KEY_SCAN;
}




/* end of key.c */
