#ifndef __MPUIIC_H
#define __MPUIIC_H
#include "main.h"
#include "mytask.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEKս��STM32������V3
//MPU6050 IIC���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2015/1/17
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2009-2019
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////
 	   		   
//IO�������� ����SDA-PB7Ϊ����������
#define MPU_SDA_IN()  {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)8<<28;}
#define MPU_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=(uint32_t)3<<28;}




////IO��������	 
//#define MPU_IIC_SCL    PBout(10) 		//SCL
//#define MPU_IIC_SDA    PBout(11) 		//SDA	 
//#define MPU_READ_SDA   PBin(11) 		//����SDA 

//IO��������	 
#define MPU_IIC_SCL_Hige    	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_SET)//����SCL�ߵ�ƽ
#define MPU_IIC_SCL_Low      	HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_RESET)//����SCL�͵�ƽ
#define MPU_IIC_SDA_Hige      HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_SET)   //����SDA�ߵ�ƽ
#define MPU_IIC_SDA_Low       HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_RESET) //����SDA�͵�ƽ
  	 
#define MPU_READ_SDA          HAL_GPIO_ReadPin(SDA_GPIO_Port,SDA_Pin)    //��SDA��ƽ

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(uint8_t txd);			//IIC����һ���ֽ�
uint8_t MPU_IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
uint8_t MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);					//IIC����ACK�ź�
void MPU_IIC_NAck(void);				//IIC������ACK�ź�
void IMPU_IC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t MPU_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);	  
#endif
















