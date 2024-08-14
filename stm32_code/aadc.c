#include "aadc.h"

float adcGetBatteryVoltage(void)
{
	HAL_Delay(10);
	HAL_ADCEx_Calibration_Start(&hadc2);
	HAL_ADC_Start(&hadc2);//启动ADC转化
	if(HAL_OK == HAL_ADC_PollForConversion(&hadc2,50))//等待转化完成、超时时间50ms
	return (float)HAL_ADC_GetValue(&hadc2)/4096*3.3*5;//计算电池电压
	
	return -1;
}

void show_battery()
{
	if(uwTick - fang.battery_cpunt < 5000)return;
		fang.battery_cpunt = uwTick;
	
	float battery_val = 0;
	
	battery_val = adcGetBatteryVoltage();
	
	if(battery_val >= 2.5f)
		battery_val = 2.5f;
	
	if((battery_val*1000/2.5f) > 0 && (battery_val*1000/2.5f) < 25)
	{
		OLED_ShowCHinese1(104,0,0);
	}
	
	if((battery_val*1000/2.5f)> 25&& (battery_val*1000/2.5f) < 50)
	{
		OLED_ShowCHinese1(104,0,1);
	}
	
	if((battery_val*1000/2.5f) > 50 && (battery_val*1000/2.5f) < 75)
	{
		OLED_ShowCHinese1(104,0,2);
	}
	
	if((battery_val*1000/2.5f) > 75 && (battery_val*1000/2.5f) < 100)
	{
		OLED_ShowCHinese1(104,0,3);
	}
	
	if((battery_val*1000/2.5f) > 100)
	{
		OLED_ShowCHinese1(104,0,3);
	}
//	OLED_ShowNum(80,0,(battery_val*1000/2.4f),3,12);
	
}

