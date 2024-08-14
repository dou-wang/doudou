/**
  *****************************************************************************
  * @file			led.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/







#include "led.h"
#include "main.h"



/* 可修改部分 */
//=========================================================
#define BRIGHTNESS_MAX	19 /* 最大亮度等级 */
#define BRIGHTNESS_MIN	0  /* 最小亮度等级 */
#define USE_TIM6_AS_LED_TIMER	1
//====
#define TIM6_ARR	1000 /* 1ms */
#define TIM6_PRE	84
//====
_LED led0={R,GPIOD,GPIO_Pin_3};
_LED led1={G,GPIOD,GPIO_Pin_0};
_LED led2={B,GPIOD,GPIO_Pin_1};
//=========================================================



void led_Config(_LED *L)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	if(L->GPIOx == GPIOA)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	if(L->GPIOx == GPIOB)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	if(L->GPIOx == GPIOC)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	if(L->GPIOx == GPIOD)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	if(L->GPIOx == GPIOE)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	if(L->GPIOx == GPIOF)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	if(L->GPIOx == GPIOG)	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=L->Pin;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;

	GPIO_Init(L->GPIOx,&GPIO_InitStruct);
}




void led_Init(void)
{
	LED_INIT;
	
	R_OFF;
	G_OFF;
	B_OFF;
	
	#if USE_TIM6_AS_LED_TIMER
		
		TIM6_Init(TIM6_ARR,TIM6_PRE);
	
	#endif
}




/* ctrl on off */
u8 brightness[LED_NUM]={0}; /* 初始亮度值设为0 */

void led_state_ctrl(void)
{
	u8 i;
	static u8 cnt[LED_NUM];
	
	for(i=0; i<LED_NUM; i++)
	{
		if(cnt[i] < brightness[i])
		{
			switch(i)
			{
				/****************************/
				case 0 : R_ON; break;
				case 1 : G_ON; break;
				case 2 : B_ON; break;
				/****************************/
			}
		}
		else
		{
			switch(i)
			{
				/****************************/
				case 0 : R_OFF; break;
				case 1 : G_OFF; break;
				case 2 : B_OFF; break;
				/****************************/
			}
		}
		
		if(cnt[i]++ > BRIGHTNESS_MAX)
		{
			cnt[i] = 0;
		}
	}
}




/* breath ctrl */
u8 led_breath(u8 led_num) /* 增大/减小亮度等级来控制呼吸 */
{
	u8 breath_finished=0;
	static u8 step[LED_NUM];
	
	switch(step[led_num])
	{
		case 0:
		{
			brightness[led_num] += 1;
			
			if(brightness[led_num] >= BRIGHTNESS_MAX)
			{
				step[led_num] = 1;
			}
			
			break;
		}
		
		case 1:
		{
			brightness[led_num] -= 1;
			
			if(brightness[led_num] <= BRIGHTNESS_MIN)
			{
				step[led_num] = 2;
			}
			
			break;
		}
		
		case 2:
		{
			brightness[led_num] = 0;
			
			step[led_num] = 0;
			breath_finished = 1;
		}
		
		default :
		{
			step[led_num] = 0;
			break;
		}
	}
	
	return breath_finished;
}




/* flash ctrl */
float time_cnt[LED_NUM]={0};
u8 flash_time_cnt[LED_NUM]={0};
/** 
	@param \\ 灯序号  亮度  闪烁次数  亮的时间  灭的时间  间断时间  函数执行周期ms \\ 
**/
u8 led_flash(u8 led_num,u8 led_brightness,u8 led_flash_time,float led_on_time,float led_off_time,float led_gap_time,float T)
{
	u8 flash_finished=0;

	if(led_brightness > BRIGHTNESS_MAX)
	{
		led_brightness = BRIGHTNESS_MAX;
	}
	
	if(flash_time_cnt[led_num] < led_flash_time) /* flashing */
	{
		if(time_cnt[led_num] < led_on_time)
		{
			brightness[led_num] = led_brightness; /* ON */
		}
		else if(time_cnt[led_num] < (led_on_time+led_off_time))
		{
			brightness[led_num] = 0; /* OFF */
		}
		else
		{
			time_cnt[led_num] = 0;
			flash_time_cnt[led_num]++;
		}
	}
	
	else /* flash gap */
	{
		if(time_cnt[led_num] < led_gap_time)
		{
			brightness[led_num] = 0;
		}
		else
		{
			time_cnt[led_num] = 0;
			flash_time_cnt[led_num] = 0;
			
			flash_finished = 1;
		}
	}
	
	time_cnt[led_num] += T;
	
	return flash_finished;
}




/* led task */
u8 led_lightmode=0; /* 控制led模式 */
void led_task(float T)
{
	led_lightmode_check(); /* 检测模式是否变化 */
	
	if(FLAG.Openmv_err)
	{
		led_lightmode = 1;
	}
	else
	{
		led_lightmode = 0;
	}
	
	switch(led_lightmode)
	{
		case 0:
		{
			led_flash(G,BRIGHTNESS_MAX,1,100,100,2500,T);
			brightness[R] = 0;
			brightness[B] = 0;
			break;
		}
		case 1:
		{
			led_flash(R,BRIGHTNESS_MAX,1,100,100,500,T);
			brightness[G] = 0;
			brightness[B] = 0;
			break;
		}
		default:break;
	}
}




/* mode change check */
u8 led_lightmode_old=0;

void led_lightmode_check(void)
{
	if(led_lightmode != led_lightmode_old)
	{
		led_lightmode_old = led_lightmode;
		led_reset();
	}
}




/* reset */
void led_reset(void)
{
	u8 i;
	
	for(i=0; i<LED_NUM; i++)
	{
		time_cnt[i] = 0;
		brightness[i] = 0;
		flash_time_cnt[i] = 0;
	}
}




#if USE_TIM6_AS_LED_TIMER

/* led timer */
void TIM6_Init(u16 arr,u16 pre)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,ENABLE);
	
	TIM_TimeBaseInitStruct.TIM_Period=arr-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=pre-1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6,&TIM_TimeBaseInitStruct);
	
	NVIC_SetPriority(TIM6_DAC_IRQn,NVIC_EncodePriority(3,2,3));
	NVIC_EnableIRQ(TIM6_DAC_IRQn);
	
	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);
	
	TIM_Cmd(TIM6,ENABLE);
}




/* Interrupt */
void TIM6_DAC_IRQHandler(void)
{
	static u32 cnt=0;
	
	if(TIM_GetITStatus(TIM6,TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
		
		cnt++;
		
		led_state_ctrl();
		
		if(cnt % 50 == 1) /* 50ms的任务 */
		{
			led_task(50);
		}
	}
}



#endif



/* end of led.c */

