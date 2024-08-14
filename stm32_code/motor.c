#include "motor.h"

/**
 *��������PID_init
 *���������õ������ת�Լ�ռ�ձ�
 *���룺int16_t motor1,int16_t motor2,ccr
 *�������
*/

void motor_set(int16_t motor1,int16_t motor2)
{
	if(motor1>999)	motor1 = 999;					//�޷�
	if(motor2>999)	motor2 = 999;					//�޷�
	
	if(motor1<0) TIM2->CCR3 = -motor1;					//����pwm��װ��
	else	TIM2->CCR3 = motor1;
	
	if(motor2<0) TIM2->CCR3 = -motor2;					//����pwm��װ��
	else TIM2->CCR3 = motor2;
}

/**
 * ������:Motor_Left_DIR
 * ����:���ֵ�����ƺ���
 * ����:Direction=(FORWARD,BACK,STOP)
 * ���:��
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
 * ������:Motor_Right_DIR
 * ����:���ֵ�����ƺ���
 * ����:Direction=(FORWARD,BACK,STOP)
 * ���:��
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
 *��������motor_enable
 *������ʹ�ܵ��
 *���룺��
 *�������
*/
void motor_enable()
{
	Flag.Is_Enable = 1;
	
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
}

/**
 *��������motor_disable
 *������ʧ�ܵ��
 *���룺��
 *�������
*/
void motor_disable()
{
	Flag.Is_Enable = 0;
	
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);	
}

/**************************************************************************************************************
*������:Read_Encoder()
*����:��ȡ������ֵ(����С����ǰǰ�����ٶ�)
*�β�:(u8 TIMX):xΪ������1����2
*����ֵ:��
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
 *��������Clear_Encoder
*�����������������ֵ�ĸ���
 *���룺��
 *�������
*/
void Clear_Encoder()
{
	Param.Sigma_Motor1Pluse = Param.Sigma_Motor2Pluse = 0;
	Param.Distance_Motor1Curret = Param.Distance_Motor2Curret = 0;
	Param.Distance_averageCurret = 0;
	Param.Distance = 0;
}

/**
 *��������Calculate_Distance
*�������������
 *���룺��
 *�������
*/
void Calculate_Distance()
{
	 Param.Sigma_Motor1Pluse += Param.UnitTime_Motor1Pluse;		//�����ۼ�����
	  Param.Sigma_Motor2Pluse += Param.UnitTime_Motor2Pluse;		//�����ۼ�����
	  
//	  if(Param.Sigma_Motor1Pluse != Param.Sigma_Motor2Pluse)		//�����ڳ��ִ򻬻���ѭ����ɵ����������岻���ʱ���¸�ֵʹ�����
//		  Param.Sigma_Motor1Pluse =  Param.Sigma_Motor2Pluse;
	  
	  Param.Distance_Motor1Curret = (Param.Sigma_Motor1Pluse/onecirclepluse)*circumferential;		//�����ۼƾ���
	  Param.Distance_Motor2Curret = (Param.Sigma_Motor2Pluse/onecirclepluse)*circumferential;		//�����ۼƾ���
	  
	  Param.Distance_averageCurret =  (Param.Distance_Motor1Curret + Param.Distance_Motor2Curret)/2;	//�����ܾ���
}

/**
 *��������Turn_CAR
 *������С��ת��
 *���룺��
 *�������
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
 *��������Car_Tracking
 *������ѭ����������
 *���룺TargetDistanceĿ�����
 *�������
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
	
	//1.��Ŀ����븳ֵ��Ŀ�������ֵ
	pid.Location_Target_val = TargetDistance;
	
	//2.ʹ�ܵ��
	motor_enable();
}

