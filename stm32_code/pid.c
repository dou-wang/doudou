#include "pid.h"


LocPID Angle_Ring;

/**
 *PID_init
 *������pid������ʼ��
*/
void PID_init()
{
	Angle_Ring.P = 50.0f;
	Angle_Ring.I = 0.0f;
	Angle_Ring.D = 14.0f;
}

/**************************************************************************
�������ܣ�����PID������
��ڲ�����ʵ��ֵ
����  ֵ��pid_param->out
��������ʽ��ɢPID��ʽ 
pid.Zeng_Actual_val += Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pid.Zeng_Actual_val�����������
**************************************************************************/
int Incremental_PID(float Zeng_Target_val,float Zeng_Actual_val,ZengPID* pid_param)
{ 	
    //1.����ƫ��
	pid_param->Err = Zeng_Target_val - Zeng_Actual_val;
	
	//2.PID�㷨��ʵ��
	pid_param->out += pid_param->P*(pid_param->Err - pid_param->Last_Err) + pid_param->I*pid_param->Err + pid_param->D*(pid_param->Err - 2*pid_param->Last_Err + pid_param->Previous_Err);
	
	//3.ƫ��Ĵ���
	pid_param->Previous_Err = pid_param->Last_Err;
	pid_param->Last_Err = pid_param->Err;
	
	//4.����λ�û�����õ������ֵ
	return pid_param->out;
}

/**
 *��������LocationRing_PID
 *������λ�û�pid����
 *���룺
 *�����pid_param ->outλ�û������ֵ
*/
int LocationRing_PID(float Location_Target_val,float Location_Actual_val,LocPID* pid_param)
{
	//1.����ƫ��
	pid_param->Err = Location_Target_val - Location_Actual_val;

	//2.�ۼ�ƫ��
	pid_param->Sum_Err += pid_param->Err;
	
	//3.PID�㷨��ʵ��
	pid_param ->out = pid_param->P*pid_param->Err + pid_param->I*pid_param->Sum_Err + pid_param->D*(pid_param->Err - pid_param->Last_Err);
	
	//4.ƫ��Ĵ���
	pid_param->Last_Err = pid_param->Err;
	
	//5.����λ�û�����õ������ֵ
	return pid_param ->out;
}



