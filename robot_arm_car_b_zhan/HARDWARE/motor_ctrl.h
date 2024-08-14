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




/* 贝塞尔曲线坐标设置 */
typedef struct
{
	u8 STEPS; /* 阶次 */
	
	u8 AXIS; /* 主轴 */
	
	float ANGLE; /* 目标角度 */
	
	_XY POS0; /* 起点 */
	
	_XY POS1; /* 控制点1 */
	
	_XY POS2; /* 控制点2 - 终点 */
	
	_XY POS3; /* 终点 */
	
}_BEZIER_POS;





/* 折线路径坐标设置 */
typedef struct
{
	u8 STEPS; /* 阶次 - 弯折次数 */
	
	u8 AXIS; /* 主轴 */
	
	float ANGLE; /* 目标角度 */
	
	_XY POS0; /* 坐标 */
	
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
