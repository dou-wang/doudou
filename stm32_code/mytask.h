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

//#define KEY0 	HAL_GPIO_ReadPin(KEY0_GPIO_Port,KEY0_Pin)						//��ȡ����KEY0
//#define KEY1	HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)						//��ȡ����KEY1

typedef struct{
	float   Sigma_Motor1Pluse;				//�����ۼ��������
	float   Sigma_Motor2Pluse;				//�ҵ���ۼ��������
	float   UnitTime_Motor1Pluse;				//���ֵ�λʱ�����������				
	float   UnitTime_Motor2Pluse;				//���ֵ�λʱ�����������	
	float   Distance_Motor1Curret;			//���־���
	float   Distance_Motor2Curret;			//���־���
	float   Distance_averageCurret;			//�����ߵ��ܣ�ƽ��������
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
	uint8_t Is_Enable;					//���ʹ�ܻ�ʧ�ܱ�־λ
	uint8_t Start_Line;					//��ʼѲ�߱�־λ
	uint8_t Stop_Car;						//С��ͣ����־λ
	uint8_t Run_Step;						//С�����б�־λ
	uint8_t Start_Spin;					//��ʼת���־λ
	uint8_t Success_Spin;					//�ɹ�ת���־λ
}Flag_InitTypeDef;

void xunji(void);
void init_param(void);
void key_scan(void);
void Car_Tracking(uint16_t TargetDistance);
extern Param_InitTypeDef Param;
extern Flag_InitTypeDef Flag;

#endif
