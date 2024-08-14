#include "encoder.h"
#include "sys.h"
#include "usart2.h"
 //使用的定时器
 //TIM1 3 4 8
 pid_type_def motor_speed_pid;
pid_type_def motor_angle_pid;
 
 float pai=3.1415926;
int16_t enc =0,target1 =500,pwm = 0,pwm1 = 0;
 
 void motor_control_init(void)
{
	static const fp32 Motor_Speed_pid[3] = {Motor_Speed_PID_KP, Motor_Speed_PID_KI, Motor_Speed_PID_KD};
	static const fp32 Motor_Angle_pid[3] = {Motor_Angle_PID_KP, Motor_Angle_PID_KI, Motor_Angle_PID_KD};
	
  PID_init(&motor_speed_pid, PID_POSITION, Motor_Speed_pid, Motor_Speed_PID_MAX_OUT, Motor_Speed_PID_MAX_IOUT);
	PID_init(&motor_angle_pid, PID_POSITION, Motor_Angle_pid, Motor_Angle_PID_MAX_OUT, Motor_Angle_PID_MAX_IOUT);
	
	PID_clear(&motor_speed_pid);
	PID_clear(&motor_angle_pid);
}

void encoder_Init(void)
{
    GPIO_InitTypeDef         GPIO_InitStructure; 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef        TIM_ICInitStructure;
 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);//开启TIM4时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//开启GPIOD时钟
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//开启TIM3时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);//开启GPIOA时钟
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);//开启TIM3时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);//开启GPIOE时钟
	
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);//开启TIM3时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);//开启GPIOC时钟
  
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource12,GPIO_AF_TIM4);//PB0引脚复用
    GPIO_PinAFConfig(GPIOD,GPIO_PinSource13,GPIO_AF_TIM4);//PB1引脚服用
	
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF_TIM2);//PB0引脚复用
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource15,GPIO_AF_TIM2);//PB1引脚服用
	
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF_TIM3);//PB0引脚复用
    GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_TIM3);//PB1引脚服用
		
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5);//PB0引脚复用
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_TIM5);//PB1引脚服用
  /*--------------------------------------------------------------*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12|GPIO_Pin_13; //GPIOD12,GPIOD13
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOD,&GPIO_InitStructure); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_15; //GPIOA1.2.15
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOA,&GPIO_InitStructure); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5|GPIO_Pin_3; //GPIOB4.5.6
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOB,&GPIO_InitStructure); 
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7; //GPIOC6.7
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP ;
    GPIO_Init(GPIOC,&GPIO_InitStructure); 

  /*--------------------------------------------------------------*/
	
    TIM_TimeBaseStructure.TIM_Period = 65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
 
    TIM_TimeBaseStructure.TIM_Period =65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); 
 
    TIM_TimeBaseStructure.TIM_Period =65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 
		
		TIM_TimeBaseStructure.TIM_Period = 65535; //设置下一个更新事件装入活动的自动重装载寄存器周期的值
    TIM_TimeBaseStructure.TIM_Prescaler = 0; //设置用来作为TIMx时钟频率除数的预分频值  不分频
    TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
/*--------------------------------------------------------------*/
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;  //输入滤波器
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM4, TIM_FLAG_Update);  //清楚所有标志位
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE); //允许中断更新
    TIM4->CNT = 0;
    TIM_Cmd(TIM4, ENABLE);  //使能TIM3
		
		TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;  //输入滤波器
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM3, TIM_FLAG_Update);  //清楚所有标志位
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE); //允许中断更新
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);  //使能TIM3
		
		TIM_EncoderInterfaceConfig(TIM2, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;  //输入滤波器
    TIM_ICInit(TIM2, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);  //清楚所有标志位
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE); //允许中断更新
    TIM2->CNT = 0;
    TIM_Cmd(TIM2, ENABLE);  //使能TIM3
		
		TIM_EncoderInterfaceConfig(TIM5, TIM_EncoderMode_TI12,TIM_ICPolarity_BothEdge,TIM_ICPolarity_BothEdge);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 10;  //输入滤波器
    TIM_ICInit(TIM5, &TIM_ICInitStructure);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);  //清楚所有标志位
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE); //允许中断更新
    TIM5->CNT = 0;
    TIM_Cmd(TIM5, ENABLE);  //使能TIM3
}


EncoderStructure1 EncoderStructure;
void Read_Encoder(void)
{
int i;

	 EncoderStructure.Encoder_Value[0]=TIM2 -> CNT;	  //正数
	 EncoderStructure.Encoder_Value[1]=TIM3 -> CNT;	
	 EncoderStructure.Encoder_Value[2]=TIM4 -> CNT;	
	 EncoderStructure.Encoder_Value[3]=TIM5 -> CNT;
//清零
	TIM2 -> CNT=0;TIM3 -> CNT=0;
	TIM4 -> CNT=0;TIM5 -> CNT=0;
	
	for(i=0;i<=3;i++)
	{
		if(EncoderStructure.Encoder_Value[i]>6000)
		{
		  EncoderStructure.Encoder_Value[i]=EncoderStructure.Encoder_Value[i]-65535;  //负数
		}
	}
  // 屏蔽小波动
	for(i=0;i<=3;i++)
	{
		if(EncoderStructure.Encoder_Value[i]<=2&&EncoderStructure.Encoder_Value[i]>=-2)
		{EncoderStructure.Encoder_Value[i]=0;
		}
	}

}



 WheelStructure1  WheelStructure;

//10ms中断一次然后测速
//计数CNT  单项极对数为11对，产生11个脉冲 ,两个对极，双边检测，因此单旋转一圈产生44个脉冲
//实际测量旋转一圈旋转的脉冲数为
//217 214 216 214 214 215 214 212 212 218 取众数214


#define Dt_10ms 1e-3
void EnANWh_Velocity(void)
{
	u8 i;
  //读取编码器的值
	EncoderStructure.Encoder_Value[0]=TIM2 -> CNT; 
	EncoderStructure.Encoder_Value[1]=TIM3 -> CNT;
	EncoderStructure.Encoder_Value[2]=TIM4 -> CNT;
	EncoderStructure.Encoder_Value[3]=TIM5 -> CNT;
	
	//大端取反得负数
	for(i=0;i<=3;i++)
		{
			if(EncoderStructure.Encoder_Value[i]>6000)
			{
				EncoderStructure.Encoder_Value[i]=EncoderStructure.Encoder_Value[i]-65535;  
			}
		}	
	
	//清零
	TIM2 -> CNT=0;TIM3 -> CNT=0;
	TIM4 -> CNT=0;TIM5 -> CNT=0;
	
	// 屏蔽小波动
	for(i=0;i<=3;i++)
	{
		if(EncoderStructure.Encoder_Value[i]<=2&&EncoderStructure.Encoder_Value[i]>=-2)
		{
			EncoderStructure.Encoder_Value[i]=0;	
		}
	}
	
	
	//得到旋转的圈数
		for(i=0;i<=3;i++)
	{
	EncoderStructure.Motor_CylNum[i]=EncoderStructure.Encoder_Value[i]/214;
	
	//得到电机的角速度,车轮的角速度，减速比为10
  EncoderStructure.Motor_angVel[i]=EncoderStructure.Motor_CylNum[i]*200*pai; //单位（rad/s） 
  WheelStructure.Wheel_angVel[i]=EncoderStructure.Motor_angVel[i]/10	;	
	
  //得到车轮的线速度
  WheelStructure.Wheel_linVel[i]=WheelStructure.Wheel_angVel[i]*WheelStructure.wheel_R;
		
	//得到车轮的行驶距离
WheelStructure.Wheel_distance[i]=WheelStructure.wheel_perimeter*EncoderStructure.Motor_CylNum[i]/10;
	}
	
}

//低通滤波器
//float L_filter(float value,float a)
//{
//	static float Last_value;
//	value=Last_value*(256-a)/256+a*value/256;
//	Last_value=value;
//	return value;
//}

#define TIM9_arr 839
#define TIM9_psc 999
//得到快关频率 100hz
#define TIM1_arr 839
#define TIM1_psc 999
//车轮速度专用解算定时器
//10ms中断一次

void TIM9_EnAnWh_Vel(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE);  ///使能TIM2时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = TIM9_arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=TIM9_psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM9,&TIM_TimeBaseInitStructure);//初始化TIM9
	
	TIM_ITConfig(TIM9,TIM_IT_Update,ENABLE); //允许定时器2更新中断
	TIM_Cmd(TIM9,ENABLE); //使能定时器9
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM1_BRK_TIM9_IRQn; //定时器2中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=2; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
void TIM1_CALL_BACK(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  ///使能TIM1时钟

  TIM_TimeBaseInitStructure.TIM_Period = TIM1_arr;  // 自动重装载值
  TIM_TimeBaseInitStructure.TIM_Prescaler = TIM1_psc;  // 定时器分频
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
  TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure); // 初始化TIM1

  TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE); // 允许定时器1更新中断
  TIM_Cmd(TIM1, ENABLE); // 使能定时器1

  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_TIM10_IRQn; // 定时器1更新中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; // 抢占优先级3
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1; // 子优先级3
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}




s16 TIM9_flag;
void  TIM1_BRK_TIM9_IRQHandler(void)
{
    if(TIM_GetITStatus(TIM9,TIM_IT_Update)!=RESET)
    {  
			
			Read_Encoder();
			enc += EncoderStructure.Encoder_Value[0];
			pwm = PID_calc(&motor_angle_pid,enc,target1);//位置环
		  pwm = PID_calc(&motor_speed_pid,EncoderStructure.Encoder_Value[0],10);//速度环
      pwm1=pwm;
			coty(pwm);
			pwm=abs_1(pwm);
			TIM8->CCR1=pwm;
			MOTOR2_SPEED=1000;
			MOTOR3_SPEED=pwm;
			MOTOR4_SPEED=pwm;
	printf("距离的值是 t0 = %d\r\n",MOTOR1_SPEED);
	printf("距离的值是 t1 = %d\r\n",pwm1);
	printf("距离的值是 t2 = %d\r\n",EncoderStructure.Encoder_Value[0]);
	printf("距离的值是 t3 = %d\r\n",TIM8->CCR1);
       TIM_ClearITPendingBit(TIM9,TIM_IT_Update); 
		}
}
void TIM1_UP_TIM10_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
    {
       
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

int16_t abs_1(int16_t x) {
  return (x < 0) ? -x : x;
}

void coty(int16_t t){
				if(t < 0){MOTOR1_R;}else{
				 MOTOR1_L;}
} 
