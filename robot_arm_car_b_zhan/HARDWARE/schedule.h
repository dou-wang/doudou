#ifndef __SCHEDULE_H
#define __SCHEDULE_H
#include "sys.h"


typedef struct
{
	void(*task_func)(void);
	
	u16 rate_hz;
	u16 interval_ticks;
	u32 last_run;
	
}_SCHEDULER_TASK;


void Scheduler_Setup(void);
void Scheduler_Run(void);




#endif

