#include "TIM8.h"

void TIM8_PWM_Init(u16 arr,u16 psc)//TIM3 PWM��� 
{		 					 
	//�˲������ֶ��޸�IO������
	RCC->APB2ENR|=1<<13;   //TIM8ʱ��ʹ��     
	RCC->APB2ENR|=1<<4;   //ʹ��PORTC��ʱ��  
	
	GPIOC->CRL&=0X00FFFFFF;	//PC6 7���
	GPIOC->CRL|=0XBB000000;	//���ù������ 	  	 
	   
	GPIOC->CRH&=0XFFFFFF00;	//PC 8 9���
	GPIOC->CRH|=0X000000BB;	//���ù������
	
	GPIOC->ODR|=15<<6;	   	//PA0 ����  

	TIM8->ARR=arr;//�趨�������Զ���װֵ  
	TIM8->PSC=psc;//Ԥ��Ƶ������Ƶ 

	TIM8->CCMR1|=7<<4;   //CH1 PWM2ģʽ	   
	TIM8->CCMR1|=1<<3;   //CH1Ԥװ��ʹ�� 
	
	TIM8->CCMR1|=7<<12;  //CH2 PWM2ģʽ	   
	TIM8->CCMR1|=1<<11;  //CH2Ԥװ��ʹ��
	
	TIM8->CCMR2|=7<<4;   //CH3 PWM2ģʽ	   
	TIM8->CCMR2|=1<<3;   //CH3Ԥװ��ʹ�� 
	
	TIM8->CCMR2|=7<<12;  //CH4 PWM2ģʽ	   
	TIM8->CCMR2|=1<<11;  //CH4Ԥװ��ʹ��	

	TIM8->CCER|=1<<0;   //OC1 ���ʹ�� 
	TIM8->CCER|=1<<1;   //OC1 ������� 
	
	TIM8->CCER|=1<<4;   //OC2 ���ʹ�� 
	TIM8->CCER|=1<<5;   //OC2 ������� 
	
	TIM8->CCER|=1<<8;   //OC3 ���ʹ�� 
	TIM8->CCER|=1<<9;   //OC3 ������� 
	
	TIM8->CCER|=1<<12;   //OC4 ���ʹ�� 
	TIM8->CCER|=1<<13;   //OC4 ������� 

	TIM8->BDTR=0X8000;	 //����PMW����� 

	TIM8->CR1|=0x01;    //ʹ�ܶ�ʱ��8 								  
}


