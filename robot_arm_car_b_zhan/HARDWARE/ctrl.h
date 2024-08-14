#ifndef __CTRL_H
#define __CTRL_H
#include "sys.h"
#include "wit_c_sdk.h"



#define ABS(x) ((x<0) ? (-x) : x)
#define POW(X) (X * X)



#define X_NOW	pos_y
#define Y_NOW	-pos_x
#define A_NOW	fAngle[2]




typedef struct
{
	u8 Init_finish  : 1;
	u8 Task_start	: 1;
	u8 Carrying		: 2;
	u8 Carry_time 	: 2;
	
	u8 Openmv_err 	: 1;
	u16 Openmv_cnt;
	
}_FLAG;




/* ��ȡ������ */
#define FIRST_X			(SECOND_X-150)
#define SECOND_X		1025
#define THIRD_X			(SECOND_X+150)
#define GET_Y			210

/* ������1���� */
#define FIRST_X_PUT		(SECOND_X_PUT-300)
#define SECOND_X_PUT	1075
#define THIRD_X_PUT		(SECOND_X_PUT+300)
#define PUT_Y			1900


/* ������2���� */
#define FIRST_Y_PUT		(SECOND_Y_PUT-300)
#define SECOND_Y_PUT	1030
#define THIRD_Y_PUT		(SECOND_Y_PUT+300)
#define PUT_X			1860


#define OPENMV_START_DIST 750



enum
{
	X=0,
	Y,
	XY,
};



enum
{
	ZONE1 = 123,
	ZONE2 = 123,
};




typedef struct
{
	u8 position_num;	/* �����ҵ�λ��ֵ */
	u8 color_num;		/* ������������ɫ˳�� */
	int get[XY]; 		/* ץȡ���� */
	int put[XY]; 		/* ������� */
	
}_COMPETETION_TASK; /* �������� */




float limit(float x,float min,float max);
void Task(int T);
void Sensor_check(int T);
void put_position_set(void);
void Second_Put_Pos_Set(void);



extern _FLAG FLAG;
extern _COMPETETION_TASK COM_Tasks[];
extern u8 OPENMV_USE_FLAG;



#endif
