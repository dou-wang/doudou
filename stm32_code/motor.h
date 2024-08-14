#ifndef __MOTOR_H_
#define __MOTOR_H_


#define AIN1_1 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define AIN1_0 HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)

#define AIN2_1 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET)
#define AIN2_0 HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET)

#define BIN1_1 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define BIN1_0 HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)

#define BIN2_1 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET)
#define BIN2_0 HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET)

#define circumferential 	16.5			//车轮周长

#define onecirclepluse 		828				//一圈的脉冲个数

#include "mytask.h"
typedef enum{
	FORWARD,
	BACK,
	STOP
}MotorDIR_Choose;


typedef enum
{
	LEFT_20,
	RIGHT_20,
	LEFT_40,
	RIGHT_40,
	LEFT_60,
	RIGHT_60,
	LEFT_90,
	RIGHT_90,
	SPIN_180
}SpinDIR_Choose;


void motor_set(int16_t motor1,int16_t motor2);
void motor_disable(void);
void motor_enable(void);
int Read_Encoder(uint8_t TIMX);
void Car_Tracking(uint16_t TargetDistance);
void Clear_Encoder(void);
void Calculate_Distance(void);
void Motor_Right_DIR(MotorDIR_Choose Direction);
void Motor_Left_DIR(MotorDIR_Choose Direction);
void Turn_CAR(SpinDIR_Choose Direction,short pwm);
#endif

