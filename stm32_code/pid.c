#include "pid.h"


LocPID Angle_Ring;

/**
 *PID_init
 *描述：pid参数初始化
*/
void PID_init()
{
	Angle_Ring.P = 50.0f;
	Angle_Ring.I = 0.0f;
	Angle_Ring.D = 14.0f;
}

/**************************************************************************
函数功能：增量PID控制器
入口参数：实际值
返回  值：pid_param->out
根据增量式离散PID公式 
pid.Zeng_Actual_val += Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pid.Zeng_Actual_val代表增量输出
**************************************************************************/
int Incremental_PID(float Zeng_Target_val,float Zeng_Actual_val,ZengPID* pid_param)
{ 	
    //1.计算偏差
	pid_param->Err = Zeng_Target_val - Zeng_Actual_val;
	
	//2.PID算法的实现
	pid_param->out += pid_param->P*(pid_param->Err - pid_param->Last_Err) + pid_param->I*pid_param->Err + pid_param->D*(pid_param->Err - 2*pid_param->Last_Err + pid_param->Previous_Err);
	
	//3.偏差的传递
	pid_param->Previous_Err = pid_param->Last_Err;
	pid_param->Last_Err = pid_param->Err;
	
	//4.返回位置环计算得到的输出值
	return pid_param->out;
}

/**
 *函数名：LocationRing_PID
 *描述：位置环pid控制
 *输入：
 *输出：pid_param ->out位置环的输出值
*/
int LocationRing_PID(float Location_Target_val,float Location_Actual_val,LocPID* pid_param)
{
	//1.计算偏差
	pid_param->Err = Location_Target_val - Location_Actual_val;

	//2.累计偏差
	pid_param->Sum_Err += pid_param->Err;
	
	//3.PID算法的实现
	pid_param ->out = pid_param->P*pid_param->Err + pid_param->I*pid_param->Sum_Err + pid_param->D*(pid_param->Err - pid_param->Last_Err);
	
	//4.偏差的传递
	pid_param->Last_Err = pid_param->Err;
	
	//5.返回位置环计算得到的输出值
	return pid_param ->out;
}



