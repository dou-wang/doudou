#include "mpuiic.h"
#include "main.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK精英STM32开发板V3
//MPU6050 IIC驱动 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2015/1/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////

/**
	* @name		delay_us
  * @brief  微秒延时函数
	* @param  us	延时微秒数   0 ~ 2^24
	* @return void
	* @Sample delay_us(2)
  */

//void mpuiic_Delayus(uint32_t us)
//{
//	us = (1<<25) <= us ? (1<<25) - 1 : us;
//	//LOAD（载入数值）= t（计时时间，单位：秒） x T（频率，单位：Hz）
//	SysTick->LOAD = us * (AHB_freq / 8000000);
//	SysTick->VAL = 0;
//	SysTick->CTRL = 0x01;
//	while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
//	SysTick->CTRL &= ~(0x01<<SysTick_CTRL_ENABLE_Pos);
//}
//void mpuiic_Delayus()  {
//    volatile unsigned char i = 2;
//    while (i)
//        i--;
//}
void mpuiic_Delayus(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay  * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:系统频率
//	OLED_ShowNumber(0,0,(SystemCoreClock /8U/1000U/1000),5,16);
  do
  {
    __NOP();//使用空指令延时、移植不同单片机注意__NOP(); 执行时间 
  }
  while (Delay--);
} 
  //MPU IIC 延时函数
void MPU_IIC_Delay(void)
{
	mpuiic_Delayus(2);
}

//初始化IIC
void MPU_IIC_Init(void)
{					     
//  GPIO_InitTypeDef  GPIO_InitStructure;
//	
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//先使能外设IO PORTB时钟 
//		
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;	 // 端口配置
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
//  GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIO 
//	
//  GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);						 //PB10,PB11 输出高	
 
}
//产生IIC起始信号
void MPU_IIC_Start(void)
{
	MPU_SDA_OUT();     //sda线输出
	MPU_IIC_SDA_Hige;	  	  
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
 	MPU_IIC_SDA_Low;//START:when CLK is high,DATA change form high to low 
	MPU_IIC_Delay();
	MPU_IIC_SCL_Low;//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MPU_IIC_Stop(void)
{
	MPU_SDA_OUT();//sda线输出
	MPU_IIC_SCL_Low;
	MPU_IIC_SDA_Low;//STOP:when CLK is high DATA change form low to high
 	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige; 
	MPU_IIC_SDA_Hige;//发送I2C总线结束信号
	MPU_IIC_Delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MPU_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MPU_SDA_IN();      //SDA设置为输入  
	MPU_IIC_SDA_Hige;MPU_IIC_Delay();	   
	MPU_IIC_SCL_Hige;MPU_IIC_Delay();	 
	while(MPU_READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			
			MPU_IIC_Stop();
			return 1;
		}
	}
	 MPU_IIC_SCL_Low;//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MPU_IIC_Ack(void)
{
	 MPU_IIC_SCL_Low;
	MPU_SDA_OUT();
	MPU_IIC_SDA_Low;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	 MPU_IIC_SCL_Low;
}
//不产生ACK应答		    
void MPU_IIC_NAck(void)
{
	 MPU_IIC_SCL_Low;
	MPU_SDA_OUT();
	MPU_IIC_SDA_Hige;
	MPU_IIC_Delay();
	MPU_IIC_SCL_Hige;
	MPU_IIC_Delay();
	 MPU_IIC_SCL_Low;
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MPU_IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
	MPU_SDA_OUT(); 	    
     MPU_IIC_SCL_Low;//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
		if((txd&0x80)>>7) MPU_IIC_SDA_Hige;
		else MPU_IIC_SDA_Low;
		
		
        txd<<=1; 	  
		    MPU_IIC_SCL_Hige;
		    MPU_IIC_Delay(); 
		     MPU_IIC_SCL_Low;	
		    MPU_IIC_Delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t MPU_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MPU_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
         MPU_IIC_SCL_Low; 
        MPU_IIC_Delay();
		MPU_IIC_SCL_Hige;
        receive<<=1;
        if(MPU_READ_SDA)receive++;   
		MPU_IIC_Delay(); 
    }					 
    if (!ack)
        MPU_IIC_NAck();//发送nACK
    else
        MPU_IIC_Ack(); //发送ACK   
    return receive;
}

























