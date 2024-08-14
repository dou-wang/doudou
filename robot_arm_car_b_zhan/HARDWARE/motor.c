#include "motor.h"

#include "main.h"


void motor_Init(u16 arr,u16 pre)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
	TIM_OCInitTypeDef TIM_OCInitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource8,GPIO_AF_TIM8);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource9,GPIO_AF_TIM8);
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	TIM_TimeBaseInitStruct.TIM_Period=arr-1;
	TIM_TimeBaseInitStruct.TIM_Prescaler=pre-1;
	TIM_TimeBaseInitStruct.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStruct.TIM_CounterMode=TIM_CounterMode_Up;
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0; // 重复计数器值为0
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

    
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
//	TIM_OCInitStruct.TIM_OCNPolarity  = TIM_OCNPolarity_High;   //必须
//	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;//必须 
//  TIM_OCInitStruct.TIM_OCNIdleState = TIM_OCIdleState_Reset; 
	TIM_OCInitStruct.TIM_Pulse=1000;
		
TIM_OC1Init(TIM8, &TIM_OCInitStruct);
TIM_OC2Init(TIM8, &TIM_OCInitStruct);
TIM_OC3Init(TIM8, &TIM_OCInitStruct);
TIM_OC4Init(TIM8, &TIM_OCInitStruct);

TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);
TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);
TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);
TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);
TIM_ARRPreloadConfig(TIM8, ENABLE);
TIM_Cmd(TIM8, ENABLE);
TIM_CtrlPWMOutputs(TIM8, ENABLE);
motor_state_Init();
}
//TIM1_PWM_Init(100-1,168-1);//168M/168=1Mhz的计数频率,重装载值100，所以PWM频率为 1M/100=10Khz. 
void TIM1_PWM_Init(u32 arr,u32 psc)
{                              
    //此部分需手动修改IO口设置
    
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);      //TIM1时钟使能    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);     //使能PORTF时钟    
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOA11复用为定时器1
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //GPIOA11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //速度100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //上拉
    GPIO_Init(GPIOA,&GPIO_InitStructure);              //初始化GPIOA11
      
    TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
    TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;  //20201020
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//初始化定时器14
    
    //初始化TIM14 Channel1 PWM模式     
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性:TIM输出比较极性低
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM1 4OC1
 
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
 
    TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPE使能 
    
    TIM_Cmd(TIM1, ENABLE);  //使能TIM1
    
     TIM_CtrlPWMOutputs(TIM1, ENABLE);//使能TIM1的PWM输出，TIM1与TIM8有效,如果没有这行会问题
 
                                          
}  

void motor_state_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOA, ENABLE); //使能端口时钟
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //G8
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);   	//初始化
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;  //G8
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;    //G4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //初始化
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;    //G3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //初始化
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;    //D14
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //初始化
	
	/* STBY1(A11) - 低电平时电机全部停转 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //初始化
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //初始化
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;    //D15
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);           	  //初始化
	
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;    //D14
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //输出
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //高速100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);           	  //初始化
	
	/* STBY2(A12) - 低电平时电机全部停转 */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	
}






void motor_on(void)
{
	MOTOR12_ALL_ON;
	MOTOR34_ALL_ON;
}



void motor_off(void)
{
	MOTOR12_ALL_OFF;
	MOTOR34_ALL_OFF;
}


/*end of motor.c*/
