#ifndef __PID_H
#define __PID_H


#include "mytask.h"
//#define pwm_max 100


typedef struct{
	float P,I,D; // 比例常数
	float Err,Last_Err,Previous_Err;
	float out;
}ZengPID;

typedef struct 
{
	float P,I,D; // 比例常数
	float Err,Last_Err;
	float Sum_Err;
	float out;
}LocPID;

//声明函数
void PID_init(void);
int Incremental_PID(float Zeng_Target_val,float Zeng_Actual_val,ZengPID* pid_param);
int LocationRing_PID(float Location_Target_val,float Location_Actual_val,LocPID* pid_param);



extern LocPID Angle_Ring;
#endif


