#ifndef _I2C_H_
#define _I2C_H_

#include "Public_StdTypes.h"
#include "system.h"

#define SDA_IN()  {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=0<<9*2;}	//PB9输入模式
#define SDA_OUT() {GPIOB->MODER&=~(3<<(9*2));GPIOB->MODER|=1<<9*2;} //PB9输出模式
//IO操作函数	 
#define IIC_SCL    PBout(8) //SCL
#define IIC_SDA    PBout(9) //SDA	 
#define READ_SDA   PBin(9)  //输入SDA 


#ifndef I2C_Direction_Transmitter
	#define  I2C_Direction_Transmitter      ((uint8_t)0x00)
#endif

#ifndef I2C_Direction_Receiver
	#define  I2C_Direction_Receiver         ((uint8_t)0x01)
#endif

enum
{
	I2C_ACK,
	I2C_NACK
};

void I2C_SDAMode(uint8_t Mode);
void I2C_Start(void);
void I2C_Stop(void);
bool I2C_WaiteForAck(void);
void I2C_Ack(void);
void I2C_NAck(void);
bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data);
bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data);
void I2C_WriteByte(uint8_t Data);
uint8_t I2C_ReadByte(uint8_t Ack);
void I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data);
uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr);
bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);
bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff);

void I2C_GPIOInit(void);

#endif

