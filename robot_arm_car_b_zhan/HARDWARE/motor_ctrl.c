/**
  *****************************************************************************
  * @file			motor_ctrl.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/





#include "motor_ctrl.h"
#include "main.h"



_PID XPID;
_PID YPID;
_PID APID;
_PID OPENMV_XPID;
_PID OPENMV_YPID;



float my_abs(float in)
{
	if(in < 0)
	{
		return -in;
	}
	else
	{
		return in;
	}
}



/* PID初始化 */
void PID_Init(void)
{
	//======================X=====================//
	XPID.P = XPID.TEMP_P = 10.0f;
	XPID.I = 0.0f;
	XPID.D = 0.0f;
	   
	XPID.I_LIMIT = 230.0f;
	XPID.OUT_LIMIT = 4200.0f;
	
	//======================Y=====================//
	YPID.P = YPID.TEMP_P = 7.5f;
	YPID.I = 0.0f;
	YPID.D = 0.0f;
	   
	YPID.I_LIMIT = 230.0f;
	YPID.OUT_LIMIT = 4200.0f;

	//======================A=====================//
	APID.P = 100.0f;
	APID.I = 0.0f;
	APID.D = 0.0f;
	  	
	APID.OUT_LIMIT = 1000.0f;
	
	//==================OPENMV-X==================//
	OPENMV_XPID.P = 12.0f;
	OPENMV_XPID.I = 0.2f;
	OPENMV_XPID.D = 0.0f;
	
	OPENMV_XPID.I_LIMIT = 300.0f;
	OPENMV_XPID.OUT_LIMIT = 500.0f;
	
	//==================OPENMV-Y==================//
	OPENMV_YPID.P = -12.0f;
	OPENMV_YPID.I = -0.2f;
	OPENMV_YPID.D = 0.0f;
	
	OPENMV_YPID.I_LIMIT = 300.0f;
	OPENMV_YPID.OUT_LIMIT = 500.0f;
}





/* X轴控制 */
short X_Ctrl(_PID *PID,float goal)
{
	PID->err = X_NOW - goal;
	
	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT =  PID->P *PID->err;
	PID->I_OUT += PID->I *PID->err;
	PID->D_OUT =  PID->D *PID->err_err;
	
	PID->I_OUT = limit(PID->I_OUT,-PID->I_LIMIT,PID->I_LIMIT);
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = limit(PID->OUT,-PID->OUT_LIMIT,PID->OUT_LIMIT); 
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}





/* Y轴控制 */
short Y_Ctrl(_PID *PID,float goal)
{
	PID->err = Y_NOW - goal;
	
	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT =  PID->P *PID->err;
	PID->I_OUT += PID->I *PID->err;
	PID->D_OUT =  PID->D *PID->err_err;
	
	PID->I_OUT = limit(PID->I_OUT,-PID->I_LIMIT,PID->I_LIMIT);
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = limit(PID->OUT,-PID->OUT_LIMIT,PID->OUT_LIMIT); 
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}





/* 角度控制 */
short A_Ctrl(_PID *PID,float goal)
{
	PID->err = A_NOW - goal;
	
	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT =  PID->P *PID->err;
	PID->I_OUT += PID->I *PID->err;
	PID->D_OUT =  PID->D *PID->err_err;
	
	PID->I_OUT = limit(PID->I_OUT,-PID->I_LIMIT,PID->I_LIMIT);
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = limit(PID->OUT,-PID->OUT_LIMIT,PID->OUT_LIMIT); 
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}



/* 摄像头精度控制 - X */
short OPENMV_Ctrl_X(_PID *PID,float goal)
{
	PID->err = Openmv.errx;
	
	if(Openmv.errx == 255)
	{
		PID->err = 0;
	}
	
	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT =  PID->P *PID->err;
	PID->I_OUT += PID->I *PID->err;
	PID->D_OUT =  PID->D *PID->err_err;
	
	PID->I_OUT = limit(PID->I_OUT,-PID->I_LIMIT,PID->I_LIMIT);
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = limit(PID->OUT,-PID->OUT_LIMIT,PID->OUT_LIMIT); 
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}


/* 摄像头精度控制 - Y */
short OPENMV_Ctrl_Y(_PID *PID,float goal)
{
	PID->err = Openmv.erry;
	
	if(Openmv.erry == 255)
	{
		PID->err = 0;
	}
	
	PID->err_err = PID->err - PID->err_old;

	PID->P_OUT =  PID->P *PID->err;
	PID->I_OUT += PID->I *PID->err;
	PID->D_OUT =  PID->D *PID->err_err;
	
	PID->I_OUT = limit(PID->I_OUT,-PID->I_LIMIT,PID->I_LIMIT);
	
	PID->OUT = PID->P_OUT + PID->I_OUT + PID->D_OUT;
	
	PID->OUT = limit(PID->OUT,-PID->OUT_LIMIT,PID->OUT_LIMIT); 
	
	PID->err_old = PID->err;
	
	return PID->OUT;
}



/* OPENMV控制精度开关 */
#define OPENMV_CTRL	1


/* 运动控制 */
int Tcnt=0; /* 摄像头定位时间记录 */
#define MAX_ERR 20 /* 最大调整误差 */
u16 time = 0;
u8 Move_Ctrl(float x_goal, float y_goal, float a_goal, u16 T)
{ 
	float X_out;
	float Y_out;
	float A_out;
	
	float rad_goal;
	
	short speed[4];
	
//	static u16 time = 0; /* 时间记录 */
	static u16 err_range = MAX_ERR; /* 调整误差范围 */
	
	rad_goal = a_goal * 0.01745f; /* 机体与世界坐标系的夹角 */
	
	X_out = X_Ctrl(&XPID,x_goal);
	Y_out = Y_Ctrl(&YPID,y_goal);
	A_out = A_Ctrl(&APID,a_goal);
	
	#if OPENMV_CTRL /* 使用OPENMV对准 */
		if(OPENMV_USE_FLAG == 1)
		{
			time += T;
			
			if(time >= 7000) /* 若7s后没扫到线，自动退出摄像头对准,改用全场定位数据放置 */ 
			{
				time = 0;
				OPENMV_USE_FLAG = 0;
			}
			
			if(my_abs(x_goal-X_NOW)<40 && my_abs(y_goal-Y_NOW)<40)
			{
				if(Openmv.change == 0)
				{
					X_out = OPENMV_Ctrl_X(&OPENMV_XPID,0);
					Y_out = OPENMV_Ctrl_Y(&OPENMV_YPID,0);
					A_out = A_Ctrl(&APID,a_goal);
				}
				else /* 侧边放置 */
				{
					Y_out = -OPENMV_Ctrl_X(&OPENMV_XPID,0);
					X_out = OPENMV_Ctrl_Y(&OPENMV_YPID,0);
					A_out = A_Ctrl(&APID,a_goal);
				}
				if(my_abs(Openmv.errx) < 100 && my_abs(Openmv.erry) < 100) /* 两条线都扫到,计时清零 */
				{
					time = 0;
				}
				if(my_abs(Openmv.errx) < 10 && my_abs(Openmv.erry) < 10)
				{
					Tcnt += T;
					
					if(Tcnt > 1000)
					{
						Tcnt = 0;
						X_out = 0;
						Y_out = 0;
						return 1;
					}
				}
			}
		}
		else
		{
			/* 到达目标位置的判断 */
			if(my_abs(x_goal-X_NOW)<100 && my_abs(y_goal-Y_NOW)<100)
			{
				time += T;
				
				if(my_abs(x_goal-X_NOW)<err_range && my_abs(y_goal-Y_NOW)<err_range)
				{
					time = 0;
					XPID.I = 0.3;
					YPID.I = 0.3;
					
					if(my_abs(x_goal-X_NOW)<6 && my_abs(y_goal-Y_NOW)<6)
					{
						X_out = 0;
						Y_out = 0;
						XPID.I_OUT = 0;
						YPID.I_OUT = 0;
						err_range = MAX_ERR; /* 归位 */
						return 1; /* 到达目标点 */
					}
				}
				else
				{
					if(time > 500) /* 误差很大 */
					{
						time = 0;
						err_range += 10; /* 扩大检测范围 */
					}
					
					XPID.I = 0.0f;
					YPID.I = 0.0f;
					XPID.I_OUT = 0;
					YPID.I_OUT = 0;
				}
			}
			else
			{
				XPID.I = 0.0f;
				YPID.I = 0.0f;
				XPID.I_OUT = 0;
				YPID.I_OUT = 0;
			}
		}
	#else
		/* 到达目标位置的判断 */
		if(my_abs(x_goal-X_NOW)<100 && my_abs(y_goal-Y_NOW)<100)
		{
			time += T;
			
			if(my_abs(x_goal-X_NOW)<err_range && my_abs(y_goal-Y_NOW)<err_range)
			{
				time = 0;
				XPID.I = 0.3;
				YPID.I = 0.3;
				
				if(my_abs(x_goal-X_NOW)<6 && my_(y_goal-Y_NOW)<6)
				{
					X_out = 0;
					Y_out = 0;
					XPID.I_OUT = 0;
					YPID.I_OUT = 0;
					err_range = MAX_ERR; /* 归位 */
					return 1; /* 到达目标点 */
				}
			}
			else
			{
				if(time > 500) /* 误差很大 */
				{
					time = 0;
					err_range += 10; /* 扩大检测范围 */
				}
				
				XPID.I = 0.0f;
				YPID.I = 0.0f;
				XPID.I_OUT = 0;
				YPID.I_OUT = 0;
			}
		}
		else
		{
			XPID.I = 0.0f;
			YPID.I = 0.0f;
			XPID.I_OUT = 0;
			YPID.I_OUT = 0;
		}
	#endif
	
	/* 世界坐标系下的控制 */
	speed[0] = +(cos(rad_goal)+sin(rad_goal))*X_out - (cos(rad_goal)-sin(rad_goal))*Y_out - A_out;
	speed[1] = -(cos(rad_goal)-sin(rad_goal))*X_out - (cos(rad_goal)+sin(rad_goal))*Y_out + A_out;
	speed[2] = -(cos(rad_goal)+sin(rad_goal))*X_out + (cos(rad_goal)-sin(rad_goal))*Y_out - A_out;
	speed[3] = +(cos(rad_goal)-sin(rad_goal))*X_out + (cos(rad_goal)+sin(rad_goal))*Y_out + A_out;
	
	/* 轮子速度控制 */
	MOTOR1_SPEED = (speed[0]);
	MOTOR2_SPEED = ABS(speed[1]);
	MOTOR3_SPEED = ABS(speed[2]);
	MOTOR4_SPEED = ABS(speed[3]);
	
	/* 轮子转向控制 */
	if(speed[0] > 0){MOTOR1_R;}
	else			{MOTOR1_L;}
	
	if(speed[1] > 0){MOTOR2_R;}
	else			{MOTOR2_L;}
	
	if(speed[2] > 0){MOTOR3_R;}
	else			{MOTOR3_L;}
	
	if(speed[3] > 0){MOTOR4_R;}
	else			{MOTOR4_L;}
	
	return 0;
}




/* 贝塞尔路径 */
u8 Bezier_Path(_BEZIER_POS bezier, u16 T)
{
	float t;
	float Bezier_outx,Bezier_outy;
	
	if(bezier.STEPS == 2)
	{
		/* 计算完成进度 */
		if(bezier.AXIS == X_AXIS)
		{
			t = my_abs(X_NOW - bezier.POS0.X) / my_abs(bezier.POS2.X - bezier.POS0.X);
		}
		if(bezier.AXIS == Y_AXIS)
		{
			t = my_abs(Y_NOW - bezier.POS0.Y) / my_abs(bezier.POS2.Y - bezier.POS0.Y);
		}
		
		/* 限值最高为1 */
		t = (t > 1) ? 1 : t;
		
		if(bezier.AXIS == X_AXIS)
		{
			Bezier_outx = bezier.POS2.X;
			Bezier_outy = (1-t)*(1-t)*bezier.POS0.Y + 2*t*(1-t)*bezier.POS1.Y + t*t*bezier.POS2.Y;
		}
		if(bezier.AXIS == Y_AXIS)
		{
			Bezier_outx = (1-t)*(1-t)*bezier.POS0.X + 2*t*(1-t)*bezier.POS1.X + t*t*bezier.POS2.X;
			Bezier_outy = bezier.POS2.Y;
		}
	}
	if(bezier.STEPS == 3)
	{
		/* 计算完成进度 */
		if(bezier.AXIS == X_AXIS)
		{
			t = my_abs(X_NOW - bezier.POS0.X) / my_abs(bezier.POS3.X - bezier.POS0.X);
		}
		if(bezier.AXIS == Y_AXIS)
		{
			t = my_abs(Y_NOW - bezier.POS0.Y) / my_abs(bezier.POS3.Y - bezier.POS0.Y);
		}
		
		/* 限值最高为1 */
		t = (t > 1) ? 1 : t;
		
		if(bezier.AXIS == X_AXIS)
		{
			Bezier_outx = bezier.POS3.X;
			Bezier_outy = (1-t)*(1-t)*(1-t)*bezier.POS0.Y + 3*t*(1-t)*(1-t)*bezier.POS1.Y + 3*t*t*(1-t)*bezier.POS2.Y + t*t*t*bezier.POS3.Y;
		}
		if(bezier.AXIS == Y_AXIS)
		{
			Bezier_outx = (1-t)*(1-t)*(1-t)*bezier.POS0.X + 3*t*(1-t)*(1-t)*bezier.POS1.X + 3*t*t*(1-t)*bezier.POS2.X + t*t*t*bezier.POS3.X;
			Bezier_outy = bezier.POS3.Y;
		}
	}
	
	return Move_Ctrl(Bezier_outx,Bezier_outy,bezier.ANGLE,T);
}



/* 折线路径 */
u8 Broken_Line_Path(_BROKEN_LINE_POS pos, u16 T)
{
	static u8 step = 0;
	static _BROKEN_LINE_POS old;
	
	/* 检测位置变化 */
	if(old.POS0.X != pos.POS0.X || old.POS0.Y != pos.POS0.Y || 
	   old.POS1.X != pos.POS1.X || old.POS1.Y != pos.POS1.Y ||
	   old.POS2.X != pos.POS2.X || old.POS2.Y != pos.POS2.Y)
	{
		step = 0;
	}
	
	old = pos;
	
	if(pos.STEPS == 1) /* 一个折点 */
	{
		if(step == 0)
		{
			if(pos.AXIS == X_AXIS) /* 先跑X轴 */
			{
				Move_Ctrl(pos.POS1.X,pos.POS0.Y,pos.ANGLE,T);
			
				if(my_abs(pos.POS1.X - X_NOW) < my_abs(pos.POS1.X - pos.POS0.X)*0.2f) /* 路径剩余20% */
					step ++;
			}
			if(pos.AXIS == Y_AXIS) /* 先跑Y轴 */
			{
				Move_Ctrl(pos.POS0.X,pos.POS1.Y,pos.ANGLE,T);
			
				if(my_abs(pos.POS1.Y - Y_NOW) < my_abs(pos.POS1.Y - pos.POS0.Y)*0.2f) /* 路径剩余20% */
					step ++;
			}
		}
		if(step == 1)
		{
			step += Move_Ctrl(pos.POS1.X,pos.POS1.Y,pos.ANGLE,T);
		}
		if(step == 2)
		{
			old = pos;
			return 1;
		}
	}
	if(pos.STEPS == 2) /* 两个折点 */
	{
		if(step == 0)
		{
			if(pos.AXIS == X_AXIS) /* 先跑X轴 */
			{
				Move_Ctrl(pos.POS1.X,pos.POS0.Y,pos.ANGLE,T);
			
				if(my_abs(pos.POS1.X - X_NOW) < my_abs(pos.POS1.X - pos.POS0.X)*0.2f) /* 路径剩余20% */
					step ++;
			}
			if(pos.AXIS == Y_AXIS) /* 先跑Y轴 */
			{
				Move_Ctrl(pos.POS0.X,pos.POS1.Y,pos.ANGLE,T);
			
				if(my_abs(pos.POS1.Y - Y_NOW) < my_abs(pos.POS1.Y - pos.POS0.Y)*0.2f) /* 路径剩余20% */
					step ++;
			}
		}
		if(step == 1)
		{
			Move_Ctrl(pos.POS1.X,pos.POS1.Y,pos.ANGLE,T);
			
			if(pos.AXIS == X_AXIS) /* 先跑X轴 */
			{
				if(my_abs(pos.POS1.Y - Y_NOW) < my_abs(pos.POS1.Y - pos.POS0.Y)*0.2f)
					step ++;
			}
			if(pos.AXIS == Y_AXIS) /* 先跑Y轴 */
			{
				if(my_abs(pos.POS1.X - X_NOW) < my_abs(pos.POS1.X - pos.POS0.X)*0.2f)
					step ++;
			}
		}
		if(step == 2)
		{
			step += Move_Ctrl(pos.POS2.X,pos.POS2.Y,pos.ANGLE,T);
		}
		if(step == 3)
		{
			return 1;
		}
	}
	
	return 0;
}



/*end of motor_ctrl.c*/
