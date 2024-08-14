#ifndef __DJ_CTRL_H
#define __DJ_CTRL_H
#include "sys.h"



enum 
{
	CIRC = 0,
	DOWN,
	MIDD,
	HIGH,
	GET,
	ACT_ALL,
};



float limit(float x,float min,float max);
void DJ_Ctrl(void);
u8 DJ_Set(short* set);



extern short DJ_ADJ[ACT_ALL];
extern short DJ_SET[3][8][6];




#endif



