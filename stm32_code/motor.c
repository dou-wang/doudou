#include "motor.h"

/**
 *函数名：PID_init
 *描述：设置电机正反转以及占空比
 *输入：int16_t motor1,int16_t motor2,ccr
 *输出：无
*/

void motor_set(int16_t motor1,int16_t motor2)
{
	if(motor1>999)	motor1 = 999;					//限幅
	if(motor2>999)	motor2 = 999;					//限幅
	
	if(motor1<0) TIM2->CCR3 = -motor1;					//进行pwm的装载
	else	TIM2->CCR3 = motor1;
	
	if(motor2<0) TIM2->CCR3 = -motor2;					//进行pwm的装载
	else TIM2->CCR3 = motor2;
}

/**
 * 函数名:Motor_Left_DIR
 * 描述:左轮电机控制函数
 * 输入:Direction=(FORWARD,BACK,STOP)
 * 输出:无
 */
void Motor_Left_DIR(MotorDIR_Choose Direction)
{
  if(FORWARD==Direction)
	{
		AIN1_1;
		AIN2_0;
	}
	else if(BACK==Direction)
	{
		AIN1_0;
		AIN2_1;	 
	}
	else if(STOP==Direction)
	{
		AIN1_0;
		AIN2_0;
	}
}

/**
 * 函数名:Motor_Right_DIR
 * 描述:右轮电机控制函数
 * 输入:Direction=(FORWARD,BACK,STOP)
 * 输出:无
 */
void Motor_Right_DIR(MotorDIR_Choose Direction)
{
  if(FORWARD==Direction)
	{
		BIN1_0;
		BIN2_1;
	}
	else if(BACK==Direction)
	{
		BIN1_1;
		BIN2_0;	 
	}
	else if(STOP==Direction)
	{
		BIN1_0;
		BIN2_0;	 
	}  
}

/**
 *函数名：motor_enable
 *描述：使能电机
 *输入：无
 *输出：无
*/
void motor_enable()
{
	Flag.Is_Enable = 1;
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

/**
 *函数名：motor_disable
 *描述：失能电机
 *输入：无
 *输出：无
*/
void motor_disable()
{
	Flag.Is_Enable = 0;
	
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);	
}

/**************************************************************************************************************
*函数名:Read_Encoder()
*功能:读取编码器值(当作小车当前前进的速度)
*形参:(u8 TIMX):x为编码器1或者2
*返回值:无
*************************************************************************************************************/
int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;  
		
	switch(TIMX)
	 {
	   case 1:  Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;
	   case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
	   default:  Encoder_TIM=0;
	 }
		return Encoder_TIM;
}


/**
 *函数名：Clear_Encoder
*描述：清零编码器的值的个数
 *输入：无
 *输出：无
*/
void Clear_Encoder()
{
	Param.Sigma_Motor1Pluse = Param.Sigma_Motor2Pluse = 0;
	Param.Distance_Motor1Curret = Param.Distance_Motor2Curret = 0;
	Param.Distance_averageCurret = 0;
	Param.Distance = 0;
}

/**
 *函数名：Calculate_Distance
*描述：计算距离
 *输入：无
 *输出：无
*/
void Calculate_Distance()
{
	 Param.Sigma_Motor1Pluse += Param.UnitTime_Motor1Pluse;		//左轮累计脉冲
	  Param.Sigma_Motor2Pluse += Param.UnitTime_Motor2Pluse;		//右轮累计脉冲
	  
//	  if(Param.Sigma_Motor1Pluse != Param.Sigma_Motor2Pluse)		//将由于车轮打滑或者循迹造成的左右轮脉冲不相等时重新赋值使其相等
//		  Param.Sigma_Motor1Pluse =  Param.Sigma_Motor2Pluse;
	  
	  Param.Distance_Motor1Curret = (Param.Sigma_Motor1Pluse/onecirclepluse)*circumferential;		//左轮累计距离
	  Param.Distance_Motor2Curret = (Param.Sigma_Motor2Pluse/onecirclepluse)*circumferential;		//右轮累计距离
	  
	  Param.Distance_averageCurret =  (Param.Distance_Motor1Curret + Param.Distance_Motor2Curret)/2;	//计算总距离
}

/**
 *函数名：Turn_CAR
 *描述：小车转弯
 *输入：无
 *输出：无
*/
void Turn_CAR(SpinDIR_Choose Direction,short pwm)
{
	Flag.Stop_Car=0;
	Flag.Start_Line=0;	
	Flag.Start_Spin=1;
	Flag.Success_Spin=0;
	Clear_Encoder();
	if(LEFT_90==Direction)
	{
		pid.Location_Target_val = 14.0f;
		Motor_Left_DIR(BACK);
		Motor_Right_DIR(FORWARD);
	}
	else if(RIGHT_90==Direction)
	{
		pid.Location_Target_val = 14.0f;
		Motor_Left_DIR(FORWARD);
		Motor_Right_DIR(BACK);	
		
	}
	else if(SPIN_180==Direction)
	{
		pid.Location_Target_val = 29.0f;
		Motor_Left_DIR(BACK);
		Motor_Right_DIR(FORWARD);  	 
	}
	
	TIM3->CCR1 = TIM3->CCR2	= pwm;
	motor_enable();	
}

/**
 *函数名：Car_Tracking
 *描述：循迹启动函数
 *输入：TargetDistance目标距离
 *输出：无
*/

void Car_Tracking(uint16_t TargetDistance)
{
//	Flag.Is_R_L = 1;
	Flag.Stop_Car=0;
	Flag.Start_Line=1;	
	Flag.Start_Spin=0;
	Flag.Success_Spin=0;
	Param.UnitTime_Motor1Pluse=0;
	Param.UnitTime_Motor2Pluse=0;
	Param.Sigma_Motor1Pluse=0;
	Param.Sigma_Motor2Pluse=0;
	
	//1.将目标距离赋值给目标距离阈值
	pid.Location_Target_val = TargetDistance;
	
	//2.使能电机
	motor_enable();
}

