#include "vl53l0x.h"
#include "vl53l0x_def.h"
#include "vl53l0x_platform.h"
#include "main.h"


_TOF TOF;


/* ��ʼ�� */
void Vl53l0x_Init(void)
{
	if(VL53L0X_Init() == VL53L0X_ERROR_NONE)
	{
		I2C_FastMode = 1;
		FLAG.TOF_err = 0;
	}
	else
	{
		FLAG.TOF_err = 1;
	}
}



/* ��� */
void Vl53_RunTask(void)
{
	static u16 err_cnt;
	
	if(FLAG.TOF_err) /* ������,Ϊ����Σ��,���ٲ�� */
	{
		TOF.Original_Dist = 1900;
		return;
	}
	
	TOF.Original_Dist = VL53L0X_FastRead();
	
	if(TOF.Original_Dist > 1900) /* ������С��1900mm */
	{
		TOF.Original_Dist = 1900;
	}
	
	if(TOF.Original_Dist == 0)
	{
		FLAG.TOF_cnt++;
		
		if(err_cnt > 10)
		{
			FLAG.TOF_err = 1;
			TOF.Original_Dist = 1900;
		}
	}
	else
	{
		FLAG.TOF_cnt = 0;
	}
	
	TOF.Relative_Dist = (TOF.Original_Dist)/10 * cos(ABS(yaw)*0.017f); /* ���ݴ���(cm) */
}


