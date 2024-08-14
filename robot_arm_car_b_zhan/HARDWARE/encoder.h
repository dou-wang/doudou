#ifndef __ENCODER_H
#define __ENCODER_H
#include "sys.h"
#include "motor.h"
#include "pid.h"
void coty(int16_t t);
 void motor_control_init(void);
void encoder_Init(void);
void Read_Encoder(void);
void EnANWh_Velocity(void);
void TIM9_EnAnWh_Vel(void);
 typedef struct {
    int Encoder_Value[4];
	  float Motor_CylNum[4];
    float Motor_angVel[4];
}EncoderStructure1;
int16_t abs_1(int16_t x);
typedef struct {
    float Wheel_angVel[4];
    float Wheel_linVel[4];
    float Wheel_distance[4];
    float wheel_R;
    float wheel_perimeter;
	  float Motor_CylNum[4];
} WheelStructure1;
extern int16_t pwm;
extern EncoderStructure1 EncoderStructure; 
#define Motor_Speed_PID_KP 10.0f 
#define Motor_Speed_PID_KI 0.3f 
#define Motor_Speed_PID_KD 0.0f 
#define Motor_Speed_PID_MAX_OUT 1500.0f
#define Motor_Speed_PID_MAX_IOUT 1500.0f

#define Motor_Angle_PID_KP -80.0f
#define Motor_Angle_PID_KI -0.0f
#define Motor_Angle_PID_KD -0.0f
#define Motor_Angle_PID_MAX_OUT 1500.0f
#define Motor_Angle_PID_MAX_IOUT 1000.0f
#endif




