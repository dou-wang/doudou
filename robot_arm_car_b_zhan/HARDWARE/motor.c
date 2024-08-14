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
	TIM_TimeBaseInitStruct.TIM_RepetitionCounter = 0; // �ظ�������ֵΪ0
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStruct);

    
	TIM_OCInitStruct.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStruct.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStruct.TIM_OutputState=TIM_OutputState_Enable;
//	TIM_OCInitStruct.TIM_OCNPolarity  = TIM_OCNPolarity_High;   //����
//	TIM_OCInitStruct.TIM_OCIdleState = TIM_OCIdleState_Set;//���� 
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
//TIM1_PWM_Init(100-1,168-1);//168M/168=1Mhz�ļ���Ƶ��,��װ��ֵ100������PWMƵ��Ϊ 1M/100=10Khz. 
void TIM1_PWM_Init(u32 arr,u32 psc)
{                              
    //�˲������ֶ��޸�IO������
    
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);      //TIM1ʱ��ʹ��    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);     //ʹ��PORTFʱ��    
    
    GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_TIM1); //GPIOA11����Ϊ��ʱ��1
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;           //GPIOA11
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;    //�ٶ�100MHz
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;        //����
    GPIO_Init(GPIOA,&GPIO_InitStructure);              //��ʼ��GPIOA11
      
    TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
    TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter=0;  //20201020
    TIM_TimeBaseInit(TIM1,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
    
    //��ʼ��TIM14 Channel1 PWMģʽ     
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
     TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //�������:TIM����Ƚϼ��Ե�
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
 
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
 
    TIM_ARRPreloadConfig(TIM1,ENABLE);//ARPEʹ�� 
    
    TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM1
    
     TIM_CtrlPWMOutputs(TIM1, ENABLE);//ʹ��TIM1��PWM�����TIM1��TIM8��Ч,���û�����л�����
 
                                          
}  

void motor_state_Init(void)
{
GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG|RCC_AHB1Periph_GPIOD|RCC_AHB1Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;  //G8
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);   	//��ʼ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;  //G8
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;    //G4
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //��ʼ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;    //G3
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //��ʼ��
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_5;    //D14
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //��ʼ��
	
	/* STBY1(A11) - �͵�ƽʱ���ȫ��ͣת */
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6;    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //��ʼ��
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8;    
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOG,&GPIO_InitStructure);           	  //��ʼ��
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;    //D15
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);           	  //��ʼ��
	
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_15;    //D14
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;           //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //���
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_100MHz;      //����100MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);           	  //��ʼ��
	
	/* STBY2(A12) - �͵�ƽʱ���ȫ��ͣת */
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
