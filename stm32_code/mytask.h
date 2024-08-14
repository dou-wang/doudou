#ifndef __MYTASK_H_
#define __MYTASK_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "gpio.h"
#include "tim.h"
#include "motor.h"
#include "pid.h"
#include "stdio.h"
#include "string.h"

//#define KEY0 	HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)						//读取按键KEY0
//#define KEY1	HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)						//读取按键KEY1

typedef struct{
	float   Sigma_Motor1Pluse;				//左电机累计脉冲个数
	float   Sigma_Motor2Pluse;				//右电机累计脉冲个数
	float   UnitTime_Motor1Pluse;				//左轮单位时间内脉冲个数				
	float   UnitTime_Motor2Pluse;				//右轮单位时间内脉冲个数	
	float   Distance_Motor1Curret;			//左轮距离
	float   Distance_Motor2Curret;			//右轮距离
	float   Distance_averageCurret;			//车身走的总（平均）距离
	float   speed;
	float   Distance;
	uint8_t Send_Step;
	uint8_t Back_Step;
}Param_InitTypeDef;

//typedef enum{
//	Wait_GetMedicine,
//	Get_Medicine,
//	send_Medicine,
//	Finish
//	
//}state_InitTypeDef;



typedef struct
{
	uint8_t Is_Enable;					//电机使能或失能标志位
	uint8_t Start_Line;					//开始巡线标志位
	uint8_t Stop_Car;						//小车停车标志位
	uint8_t Run_Step;						//小车运行标志位
	uint8_t Start_Spin;					//开始转弯标志位
	uint8_t Success_Spin;					//成功转弯标志位
}Flag_InitTypeDef;

void xunji(void);
void init_param(void);
void key_scan(void);
void Car_Tracking(uint16_t TargetDistance);
extern Param_InitTypeDef Param;
extern Flag_InitTypeDef Flag;

#endif
