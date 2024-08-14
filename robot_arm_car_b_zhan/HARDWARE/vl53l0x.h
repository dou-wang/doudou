#ifndef _VL53L0X_H_
#define _VL53L0X_H_
#include "sys.h"


typedef struct
{
	short Original_Dist; /* mm */
	short Relative_Dist; /* cm */
	short Last_Dist;
	
}_TOF;


extern _TOF TOF;



void Vl53l0x_Init(void);
void Vl53_RunTask(void);


extern _TOF tof;


#endif
