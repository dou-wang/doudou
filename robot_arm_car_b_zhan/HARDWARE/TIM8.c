#include "TIM8.h"

void TIM8_PWM_Init(u16 arr,u16 psc)//TIM3 PWM输出 
{		 					 
	//此部分需手动修改IO口设置
	RCC->APB2ENR|=1<<13;   //TIM8时钟使能     
	RCC->APB2ENR|=1<<4;   //使能PORTC口时钟  
	
	GPIOC->CRL&=0X00FFFFFF;	//PC6 7输出
	GPIOC->CRL|=0XBB000000;	//复用功能输出 	  	 
	   
	GPIOC->CRH&=0XFFFFFF00;	//PC 8 9输出
	GPIOC->CRH|=0X000000BB;	//复用功能输出
	
	GPIOC->ODR|=15<<6;	   	//PA0 上拉  

	TIM8->ARR=arr;//设定计数器自动重装值  
	TIM8->PSC=psc;//预分频器不分频 

	TIM8->CCMR1|=7<<4;   //CH1 PWM2模式	   
	TIM8->CCMR1|=1<<3;   //CH1预装载使能 
	
	TIM8->CCMR1|=7<<12;  //CH2 PWM2模式	   
	TIM8->CCMR1|=1<<11;  //CH2预装载使能
	
	TIM8->CCMR2|=7<<4;   //CH3 PWM2模式	   
	TIM8->CCMR2|=1<<3;   //CH3预装载使能 
	
	TIM8->CCMR2|=7<<12;  //CH4 PWM2模式	   
	TIM8->CCMR2|=1<<11;  //CH4预装载使能	

	TIM8->CCER|=1<<0;   //OC1 输出使能 
	TIM8->CCER|=1<<1;   //OC1 输出极性 
	
	TIM8->CCER|=1<<4;   //OC2 输出使能 
	TIM8->CCER|=1<<5;   //OC2 输出极性 
	
	TIM8->CCER|=1<<8;   //OC3 输出使能 
	TIM8->CCER|=1<<9;   //OC3 输出极性 
	
	TIM8->CCER|=1<<12;   //OC4 输出使能 
	TIM8->CCER|=1<<13;   //OC4 输出极性 

	TIM8->BDTR=0X8000;	 //设置PMW主输出 

	TIM8->CR1|=0x01;    //使能定时器8 								  
}


