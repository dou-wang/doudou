#include "schedule.h"
#include "main.h"


void Loop_1000Hz(void)
{
	
}

void Loop_500Hz(void)
{
	
}

void Loop_200Hz(void)
{
	usart1_Task();
}

void Loop_100Hz(void)
{
	Task(10);	/* 运动任务 */
}

void Loop_50Hz(void)
{ 
	key_scan(20); 	/* 按键控制 */
	Oled_Refresh(); /* 屏幕显示 */
}

void Loop_20Hz(void)
{
	Sensor_check(50); /* 传感器检测 */
}

void Loop_2Hz(void)
{
//	wifi_Init(500); /* WIFI初始化 */
}


_SCHEDULER_TASK scheduler_tasks[] = 
{
	{Loop_1000Hz,1000,  0, 0},
	{Loop_500Hz , 500,  0, 0},
	{Loop_200Hz , 200,  0, 0},
	{Loop_100Hz , 100,  0, 0},
	{Loop_50Hz  ,  50,  0, 0},
	{Loop_20Hz  ,  20,  0, 0},
	{Loop_2Hz   ,   2,  0, 0},
};




#define TASK_NUM (sizeof(scheduler_tasks)/sizeof(_SCHEDULER_TASK))


void Scheduler_Setup(void)
{
	u8 index = 0;
	
	for(index=0; index<TASK_NUM; index++)
	{
		scheduler_tasks[index].interval_ticks = 1000/scheduler_tasks[index].rate_hz; /* ???? */
		
		if(scheduler_tasks[index].interval_ticks < 1)
		{
			scheduler_tasks[index].interval_ticks = 1;
		}
	}
}

void Scheduler_Run(void)
{
	u8 index = 0;
	
	for(index=0; index<TASK_NUM; index++)
	{
		u32 T_now = Systick_Get_MS();
		
		if(T_now - scheduler_tasks[index].last_run >= scheduler_tasks[index].interval_ticks)
		{
			scheduler_tasks[index].last_run = T_now;
			
			scheduler_tasks[index].task_func();
		}	 
	}
}



