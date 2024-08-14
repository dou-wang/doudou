#ifndef __IIC_H
#define	__IIC_H
#include "sys.h"


#define SCL_H         GPIOD->BSRRL = GPIO_Pin_12
#define SCL_L         GPIOD->BSRRH = GPIO_Pin_12
                          
#define SDA_H         GPIOD->BSRRL = GPIO_Pin_13
#define SDA_L         GPIOD->BSRRH = GPIO_Pin_13
                          
#define SCL_read      GPIOD->IDR  & GPIO_Pin_12
#define SDA_read      GPIOD->IDR  & GPIO_Pin_13




void I2c_Soft_Init(void);

void I2c_Soft_SendByte(u8 SendByte);
u8 I2c_Soft_ReadByte(u8);

u8 IIC_Write_1Byte(u8 SlaveAddress,u8 REG_Address,u8 REG_data);
u8 IIC_Read_1Byte(u8 SlaveAddress,u8 REG_Address,u8 *REG_data);

u8 IIC_Write_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);
u8 IIC_Read_nByte(u8 SlaveAddress, u8 REG_Address, u8 len, u8 *buf);


extern volatile u8 I2C_FastMode;



#endif
