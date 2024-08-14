/**
  *****************************************************************************
  * @file			motor_ctrl.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/




#ifndef __MOTOR_CTRL_H
#define __MOTOR_CTRL_H
#include "sys.h"



#define X_AXIS			0
#define Y_AXIS			1

#define FIRST_ORDER		2
#define SECOND_ORDER	2
#define THIRD_ORDER		3



typedef struct
{
	float P;
	float TEMP_P;
	float P_OUT;
	
	float I;
	float I_OUT;
	float I_LIMIT;
	
	float D;
	float D_OUT;
	
	float OUT;
	float OUT_LIMIT;
	
	float err;
	float err_err;
	float err_old;
	
}_PID;




typedef struct
{
	float X;
	float Y;
	
}_XY;




/* ������������������ */
typedef struct
{
	u8 STEPS; /* �״� */
	
	u8 AXIS; /* ���� */
	
	float ANGLE; /* Ŀ��Ƕ� */
	
	_XY POS0; /* ��� */
	
	_XY POS1; /* ���Ƶ�1 */
	
	_XY POS2; /* ���Ƶ�2 - �յ� */
	
	_XY POS3; /* �յ� */
	
}_BEZIER_POS;





/* ����·���������� */
typedef struct
{
	u8 STEPS; /* �״� - ���۴��� */
	
	u8 AXIS; /* ���� */
	
	float ANGLE; /* Ŀ��Ƕ� */
	
	_XY POS0; /* ���� */
	
	_XY POS1; 
	
	_XY POS2; 
	
}_BROKEN_LINE_POS;




float my_abs(float in);
void PID_Init(void);
short X_Ctrl(_PID *PID,float goal);
short Y_Ctrl(_PID *PID,float goal);
short A_Ctrl(_PID *PID,float goal);
u8 Move_Ctrl(float x_goal, float y_goal, float a_goal, u16 T);
u8 Bezier_Path(_BEZIER_POS pos, u16 T);
u8 Broken_Line_Path(_BROKEN_LINE_POS pos, u16 T);


extern _PID XPID;
extern _PID YPID;
extern _PID APID;



#endif


/*end of motor_ctrl.h*/
