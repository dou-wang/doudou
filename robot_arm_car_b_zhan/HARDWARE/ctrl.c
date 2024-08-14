#include "ctrl.h"
#include "main.h"



_FLAG FLAG = {0};




_COMPETETION_TASK COM_Tasks[]=
	/* position_num    color_num    		 get                put			*/
{   /*	  λ��ֵ, ������ɨ����ɫ��ֵ,  ��Ӧλ������,       �������  ----  ����,��һ�������һ��ͨ������ͷʶ��ȷ�� */
			{1,			   1,   	    {FIRST_X  ,GET_Y},	{FIRST_X_PUT  ,PUT_Y}},
			{2,			   2,   	    {SECOND_X ,GET_Y},	{SECOND_X_PUT ,PUT_Y}},
			{3,			   3,   	    {THIRD_X  ,GET_Y},	{THIRD_X_PUT  ,PUT_Y}}
};  /*				   ����ͷ����       	         	  ����ͷɨ���ȷ��  	*/                     





/* �Ǻķ���Դ��ʱ */
u8 my_delay_ms(int time,int T)
{
	static int T_cnt;
	
	if(T_cnt < time) /* ��ʱ��..... */
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




/* ��ֵ */
float limit(float x,float min,float max)
{
	x = (x>max) ? max : ((x<min) ? min : x);
	return x;
}




/* ��������� */
void Sensor_check(int T)
{
	FLAG.Openmv_cnt += T;
	
	if(FLAG.Openmv_cnt   > 500)	
		FLAG.Openmv_err	 = 1;
	else
		FLAG.Openmv_err	 = 0;
}




/* ����ͷ��׼��־ */
u8 OPENMV_USE_FLAG = 0;


/* ץȡ������� */
u8 target;
u8 Get_And_Put(u8 code_num , int T)
{
	u8 num;
	static u8 step=0;
	static u8 T_Flag=0;
	static u16 f=0;
	
	FLAG.Carrying = code_num;
	
	if(T_Flag == 0) /* �ȴ�ȷ��Ŀ�� */
	{
		for(num=0; num<3; num++) /* ɨ�� */
		{ 
			if(code_num == COM_Tasks[num].color_num) /* ȷ��Ŀ������λ�� */
			{
				f = 0;
				target = num; /* ȷ�� */
				T_Flag = 1; 
				break;
			}
		}
	}
	else /* Ŀ��λ��ȷ�� */
	{
		if(step == 0) /* ��е��׼��,ǰ����ȡλ�� */
		{
			if(Openmv.change) /* �ڶ��η�ֹ��ȡ��ʱ����ײ���ϸı�Y���ٶ� */
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
		if(step == 1) /* ��ʱ */
		{
			YPID.OUT_LIMIT = 3800.0f;
			Move_Ctrl(COM_Tasks[target].get[X],COM_Tasks[target].get[Y],0, T);
			step += my_delay_ms(200,T);
		}
		if(step == 2) /* ��ȡ */
		{
			dj_set(&P3);
			Move_Ctrl(COM_Tasks[target].get[X],COM_Tasks[target].get[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 3) /* ��ס */
		{
			motor_off();
			dj_set(&P4);
			step += my_delay_ms(300,T);
		}
		if(step == 4) /* ���� */
		{	
			motor_on();
			dj_set(&P5);
			Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 5) /* ǰ������λ��/ת�� */
		{
			dj_set(&P6);
			if(Openmv.change) /* �ڶ��η�ֹ���õ�ʱ����ɨ�ı�X���ٶ� */
			{
				OPENMV_USE_FLAG = 1; /* ��������ͷ��׼���� */
				XPID.OUT_LIMIT = 2000;
			}
			step += Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
		}
		if(step == 6) /* ��ʱ */
		{
			XPID.OUT_LIMIT = 3500;
			Move_Ctrl(COM_Tasks[target].put[X],COM_Tasks[target].put[Y],0, T);
			step += my_delay_ms(300,T);
		}
		if(step == 7) /* ���� */
		{
			if(Openmv.change)
			{
				OPENMV_USE_FLAG = 0; /* �ر�����ͷ��׼���� */
			}
			motor_off();
			dj_set(&P7);
			step += my_delay_ms(300,T);
		}
		if(step == 8) /* �ſ� */
		{
			dj_set(&P8);
			step += my_delay_ms(300,T);
		}
		if(step == 9) /* �ջ� */
		{
			dj_set(&P9);
			step += my_delay_ms(300,T);
		}
		if(step == 10) /* ��� */
		{
			if(Openmv.change == 1) /* �ڶ���ɨ���ĵ�һ�μ�ȡ���ֿ��� */
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


/* ���Կ��� */
#define DEBUG	0
u8 debug_ctrl=0;

void Task(int T)
{	
	/* ���� */
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
		
	/* ���� */
	#else  
		static u8 task_step=0;
	
		if(FLAG.Task_start)
		{
			if(task_step == 0) /* ��ȥɨ�� */
			{
				/* ɨ����ɫʱ�����ٶ� */
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
					dj_set(&P1); /* ׼��ɨ����ɫ */
				}
				
				if(Code.finish == 1) /* ɨ��ɹ� */
				{
					task_step++;
				}
			}
			/*--------------------------------------------*/
			if(task_step == 1) /* ��ȡ��һ������ */
			{
				task_step += Get_And_Put(Code.first_num,T);
			}
			if(task_step == 2) /* ��ȡ�ڶ������� */
			{
				task_step += Get_And_Put(Code.second_num,T);
			}
			if(task_step == 3) /* ��ȡ���������� */
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
				
				if(Code.finish == 1) /* ɨ��ɹ� */
				{
					task_step++;
				}
			}
			/*--------------------------------------------*/
			if(task_step == 6) /* ��ȡ��һ������ */
			{
				task_step += Get_And_Put(Code.first_num,T);
			}
			if(task_step == 7) /* ��ȡ�ڶ������� */
			{
				task_step += Get_And_Put(Code.second_num,T);
			}
			if(task_step == 8) /* ��ȡ���������� */
			{
				task_step += Get_And_Put(Code.third_num,T);
			}
			/*--------------------------------------------*/
			if(task_step == 9) /* ��� */ 
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




/* �����ڶ���ȡ�Ų������� */
void put_position_set(void)
{
	/* ��е�۶������� */
	P2 = P10;
	P3 = P11;
	P4 = P12;
	P5 = P13;
	P6 = P14;
	P7 = P15;
	P8 = P16;
	P9 = P17;
	
	Openmv.change = 1; /* ����ͷ��������ת�� */
	
	COM_Tasks[0].get[X] = FIRST_X_PUT-15;
	COM_Tasks[1].get[X] = SECOND_X_PUT-15;   
	COM_Tasks[2].get[X] = THIRD_X_PUT-15;
	
	COM_Tasks[0].get[Y] = PUT_Y+10;
	COM_Tasks[1].get[Y] = PUT_Y+10;
	COM_Tasks[2].get[Y] = PUT_Y+10;
	
	COM_Tasks[0].put[X] = PUT_X;
	COM_Tasks[1].put[X] = PUT_X;
	COM_Tasks[2].put[X] = PUT_X;
	
	/* ȷ���ڶ��δ��ʱ��Ӧ��ɫ��Ӧλ�� */
	if(ZONE1 == 123) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
	{
		COM_Tasks[0].color_num = 1;
		COM_Tasks[1].color_num = 2;
		COM_Tasks[2].color_num = 3;
	}
	
	if(ZONE1 == 132) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
	{
		COM_Tasks[0].color_num = 1;
		COM_Tasks[1].color_num = 3;
		COM_Tasks[2].color_num = 2;
	}
	
	if(ZONE1 == 213) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
	{
		COM_Tasks[0].color_num = 2;
		COM_Tasks[1].color_num = 1;
		COM_Tasks[2].color_num = 3;
	}
	
	if(ZONE1 == 231) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
	{
		COM_Tasks[0].color_num = 2;
		COM_Tasks[1].color_num = 3;
		COM_Tasks[2].color_num = 1;
	}
	
	if(ZONE1 == 312) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
	{
		COM_Tasks[0].color_num = 3;
		COM_Tasks[1].color_num = 1;
		COM_Tasks[2].color_num = 2;
	}
	
	if(ZONE1 == 321) /* �ڶ��μ�ȡ�ط�����ɫ˳�� */
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */								 
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */	
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */								 
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */								 
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */								 
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
		/* ȷ����һ�δ��ʱ��Ӧ��ɫ��Ӧλ�� */
		switch (COM_Tasks[0].color_num)
		{	/* ������1˳��:��/��/�� */								 
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




