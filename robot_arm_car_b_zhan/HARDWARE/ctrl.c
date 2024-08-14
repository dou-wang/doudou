#include "ctrl.h"
#include "main.h"



_FLAG FLAG = {0};




_COMPETETION_TASK COM_Tasks[]=
	/* position_num    color_num    		 get                put			*/
{   /*	  位置值, 从左到右扫到颜色的值,  对应位置坐标,       存放坐标  ----  其中,第一个和最后一个通过摄像头识别确定 */
			{1,			   1,   	    {FIRST_X  ,GET_Y},	{FIRST_X_PUT  ,PUT_Y}},
			{2,			   2,   	    {SECOND_X ,GET_Y},	{SECOND_X_PUT ,PUT_Y}},
			{3,			   3,   	    {THIRD_X  ,GET_Y},	{THIRD_X_PUT  ,PUT_Y}}
};  /*				   摄像头决定       	         	  摄像头扫描后确定  	*/                     





/* 非耗费资源延时 */
u8 my_delay_ms(int time,int T)
{
	static int T_cnt;
	
	if(T_cnt < time) /* 延时中..... */
	{
		T_cnt += T;
		return 0;
	}
	else
	{
		T_cnt = 0;
		return 1;
	}
}




/* 限值 */
float limit(float x,float min,float max)
{
	x = (x>max) ? max : ((x<min) ? min : x);
	return x;
}




/* 传感器检测 */
void Sensor_check(int T)
{
	FLAG.Openmv_cnt += T;
	
	if(FLAG.Openmv_cnt   > 500)	
		FLAG.Openmv_err	 = 1;
	else
		FLAG.Openmv_err	 = 0;
}




/* 摄像头对准标志 */
u8 OPENMV_USE_FLAG = 0;


/* 抓取存放物料 */
u8 target;
u8 Get_And_Put(u8 code_num , int T)
{
	u8 num;
	static u8 step=0;
	static u8 T_Flag=0;
	static u16 f=0;
	
	FLAG.Carrying = code_num;
	
	if(T_Flag == 0) /* 等待确定目标 */
	{
		for(num=0; num<3; num++) /* 扫描 */
		{ 
			if(code_num == COM_Tasks[num].color_num) /* 确定目标所在位置 */
			{
				f = 0;
				target = num; /* 确认 */
				T_Flag = 1; 
				break;
			}
		}
	}
	else /* 目标位置确定 */
	{
		if(step == 0) /* 机械臂准备,前往夹取位置 */
		{
			if(Openmv.change) /* 第二次防止夹取的时候碰撞物料改变Y轴速度 */
			{
				if(Code.finish == 2)
				{
					YPID.OUT_LIMIT = 2000;
				}
				if(f == 0)
				{
					f += my_delay_ms(500,T); 
				}
				else
				{
					dj_set(&P2);
				}
			}
			else
			{
				dj_set(&P2);
			}
			step += Move_Ctrl(COM_Tasks[target].get[X],COM_Tasks[target].get[Y],0, T);
		}
		if(step == 1) /* 延时 */
		{
			YPID.OUT_LIMIT = 3800.0f;
			Move_Ctrl(COM_Tasks[target].get[X],COM_Tasks[target].get[Y],0, T);
			step += my_delay_ms(200,T);
		}
		if(step == 2) /* 夹取 */
		{
			dj_set(&P3);
			Move_Ctrl(COM_Tasks[target].get[X],COM_Tasks[target].get[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 3) /* 夹住 */
		{
			motor_off();
			dj_set(&P4);
			step += my_delay_ms(300,T);
		}
		if(step == 4) /* 缩回 */
		{	
			motor_on();
			dj_set(&P5);
			Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 5) /* 前往放置位置/转向 */
		{
			dj_set(&P6);
			if(Openmv.change) /* 第二次防止放置的时候勿扫改变X轴速度 */
			{
				OPENMV_USE_FLAG = 1; /* 开启摄像头对准靶心 */
				XPID.OUT_LIMIT = 2000;
			}
			step += Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
		}
		if(step == 6) /* 延时 */
		{
			XPID.OUT_LIMIT = 3500;
			Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 7) /* 放置 */
		{
			if(Openmv.change)
			{
				OPENMV_USE_FLAG = 0; /* 关闭摄像头对准靶心 */
			}
			motor_off();
			dj_set(&P7);
			step += my_delay_ms(300,T);
		}
		if(step == 8) /* 放开 */
		{
			dj_set(&P8);
			step += my_delay_ms(300,T);
		}
		if(step == 9) /* 收回 */
		{
			dj_set(&P9);
			step += my_delay_ms(300,T);
		}
		if(step == 10) /* 完成 */
		{
			if(Openmv.change == 1) /* 第二次扫码后的第一次夹取保持快速 */
			{
				dj_set(&P18);
				Code.finish = 2;
			}
			motor_on();
			step=0;
			T_Flag=0;
			return 1;
		}
	}
	
	return 0;
}


/* 调试开关 */
#define DEBUG	0
u8 debug_ctrl=0;

void Task(int T)
{	
	/* 调试 */
	#if DEBUG
		if(debug_ctrl == 0)
		{
			dj_set(&P14);
			Move_Ctrl(0, 0, 0, T);
		}
		if(debug_ctrl == 1)
		{
			Move_Ctrl(0, 0, 0, T);
			dj_set(&P15);
		}
		if(debug_ctrl == 2)
		{
			Move_Ctrl(0, 0, 0, T);
			dj_set(&P16);
		}
		if(debug_ctrl == 3)
		{
			Move_Ctrl(0, 0, 0, T);
			dj_set(&P17);
		}
		if(debug_ctrl == 4)
		{
			Move_Ctrl(0, 0, 0, T);
			dj_set(&P10);
		}
		
	/* 竞赛 */
	#else  
		static u8 task_step=0;
	
		if(FLAG.Task_start)
		{
			if(task_step == 0) /* 过去扫码 */
			{
				/* 扫描颜色时降低速度 */
				if(Openmv.finish)
				{
					Move_Ctrl(1600, 230, 0, T);
					XPID.OUT_LIMIT = 4200.0f;
				}
				else
				{
					Move_Ctrl(1600, 140, 0, T);
					XPID.OUT_LIMIT = 2500.0f;
				}
				
				if(X_NOW >= 200)
				{
					dj_set(&P1); /* 准备扫描颜色 */
				}
				
				if(Code.finish == 1) /* 扫描成功 */
				{
					task_step++;
				}
			}
			/*--------------------------------------------*/
			if(task_step == 1) /* 夹取第一个物料 */
			{
				task_step += Get_And_Put(Code.first_num,T);
			}
			if(task_step == 2) /* 夹取第二个物料 */
			{
				task_step += Get_And_Put(Code.second_num,T);
			}
			if(task_step == 3) /* 夹取第三个物料 */
			{
				task_step += Get_And_Put(Code.third_num,T);
			}
			/*--------------------------------------------*/
			if(task_step == 4)
			{
				Code.finish = 0;
				put_position_set();
				task_step++;
			}
			if(task_step == 5)
			{
				Move_Ctrl(1600, 230, 0, T);
				
				if(Code.finish == 1) /* 扫描成功 */
				{
					task_step++;
				}
			}
			/*--------------------------------------------*/
			if(task_step == 6) /* 夹取第一个物料 */
			{
				task_step += Get_And_Put(Code.first_num,T);
			}
			if(task_step == 7) /* 夹取第二个物料 */
			{
				task_step += Get_And_Put(Code.second_num,T);
			}
			if(task_step == 8) /* 夹取第三个物料 */
			{
				task_step += Get_And_Put(Code.third_num,T);
			}
			/*--------------------------------------------*/
			if(task_step == 9) /* 完成 */ 
			{
//				dj_set(&P0);
				if(X_NOW >= 1200)
					Move_Ctrl(0,Y_NOW,0, T);
				else
					Move_Ctrl(0,0,0, T);
			}
		}
	#endif
}




/* 初赛第二次取放参数设置 */
void put_position_set(void)
{
	/* 机械臂动作设置 */
	P2 = P10;
	P3 = P11;
	P4 = P12;
	P5 = P13;
	P6 = P14;
	P7 = P15;
	P8 = P16;
	P9 = P17;
	
	Openmv.change = 1; /* 摄像头调整坐标转换 */
	
	COM_Tasks[0].get[X] = FIRST_X_PUT-15;
	COM_Tasks[1].get[X] = SECOND_X_PUT-15;   
	COM_Tasks[2].get[X] = THIRD_X_PUT-15;
	
	COM_Tasks[0].get[Y] = PUT_Y+10;
	COM_Tasks[1].get[Y] = PUT_Y+10;
	COM_Tasks[2].get[Y] = PUT_Y+10;
	
	COM_Tasks[0].put[X] = PUT_X;
	COM_Tasks[1].put[X] = PUT_X;
	COM_Tasks[2].put[X] = PUT_X;
	
	/* 确定第二次存放时相应颜色对应位置 */
	if(ZONE1 == 123) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 1;
		COM_Tasks[1].color_num = 2;
		COM_Tasks[2].color_num = 3;
	}
	
	if(ZONE1 == 132) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 1;
		COM_Tasks[1].color_num = 3;
		COM_Tasks[2].color_num = 2;
	}
	
	if(ZONE1 == 213) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 2;
		COM_Tasks[1].color_num = 1;
		COM_Tasks[2].color_num = 3;
	}
	
	if(ZONE1 == 231) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 2;
		COM_Tasks[1].color_num = 3;
		COM_Tasks[2].color_num = 1;
	}
	
	if(ZONE1 == 312) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 3;
		COM_Tasks[1].color_num = 1;
		COM_Tasks[2].color_num = 2;
	}
	
	if(ZONE1 == 321) /* 第二次夹取地方的颜色顺序 */
	{
		COM_Tasks[0].color_num = 3;
		COM_Tasks[1].color_num = 2;
		COM_Tasks[2].color_num = 1;
	}
	
	Second_Put_Pos_Set();
}


void Second_Put_Pos_Set(void)
{
	if(ZONE2 == 123)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:红/绿/蓝 */								 
			case 1:  COM_Tasks[0].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[0].put[Y] = THIRD_Y_PUT ;break;
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[1].put[Y] = THIRD_Y_PUT ;break;
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[2].put[Y] = THIRD_Y_PUT ;break;
			default:break;
		}
	}
	 
	
	/*----------------------------132-------------------------------*/
	if(ZONE2 == 132)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:红/蓝/绿 */	
			case 1:  COM_Tasks[0].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[0].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[1].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = FIRST_Y_PUT ;break;
			case 2:  COM_Tasks[2].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}
	}
	
	
	/*----------------------------213-------------------------------*/
	if(ZONE2 == 213)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:绿/红/蓝 */								 
			case 1:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[0].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[0].put[Y] = THIRD_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[1].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[1].put[Y] = THIRD_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[2].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[2].put[Y] = THIRD_Y_PUT;break; 
			default:break;
		}
	}
	
	/*----------------------------231-------------------------------*/
	if(ZONE2 == 231)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:绿/蓝/红 */								 
			case 1:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[0].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[0].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[1].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[1].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			case 2:  COM_Tasks[2].put[Y] = THIRD_Y_PUT;break; 
			case 3:  COM_Tasks[2].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}
	}
	
	/*----------------------------312-------------------------------*/
	if(ZONE2 == 312)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:蓝/红/绿 */								 
			case 1:  COM_Tasks[0].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[0].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[1].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[2].put[Y] = FIRST_Y_PUT;break; 
			case 3:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			default:break;
		}
	}
	
	/*----------------------------321-------------------------------*/
	if(ZONE2 == 321)
	{
		/* 确定第一次存放时相应颜色对应位置 */
		switch (COM_Tasks[0].color_num)
		{	/* 放置区1顺序:蓝/绿/红 */								 
			case 1:  COM_Tasks[0].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[0].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[0].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[1].color_num)              
		{
			case 1:  COM_Tasks[1].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[1].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[1].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}                                       
		switch (COM_Tasks[2].color_num)
		{                                       
			case 1:  COM_Tasks[2].put[Y] = THIRD_Y_PUT;break; 
			case 2:  COM_Tasks[2].put[Y] = SECOND_Y_PUT;break;
			case 3:  COM_Tasks[2].put[Y] = FIRST_Y_PUT;break; 
			default:break;
		}
	}
}




