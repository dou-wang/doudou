#include "I2C.h"
/**************************************************************************
Function: IIC pin initialization
Input   : none
Output  : none
�������ܣ�IIC���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C_GPIOInit(void)
{
	
//	GPIO_InitTypeDef  GPIO_InitStructure;
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��

	
	
	IIC_SCL_1;
	IIC_SDA_1;
	

}


void delay_us(uint32_t usdelay)
{
  __IO uint32_t Delay = usdelay * (SystemCoreClock /8U/1000U/1000);//SystemCoreClock:ϵͳƵ��
  do
  {
    __NOP();//ʹ�ÿ�ָ����ʱ����ֲ��ͬ��Ƭ��ע��__NOP(); ִ��ʱ�� 
  }
  while (Delay--);
} 
/**************************************************************************
Function: Simulate IIC start signal
Input   : none
Output  : none
�������ܣ�ģ��IIC��ʼ�ź�
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C_Start(void)
{
	SDA_OUT();     //sda�����
	IIC_SCL_1;
	if(!READ_SDA)return ;	
	IIC_SCL_1;
	delay_us(1);
 	IIC_SDA_0;//START:when CLK is high,DATA change form high to low 
	if(READ_SDA)return ;
	delay_us(1);
	IIC_SCL_0;//ǯסI2C���ߣ�׼�����ͻ�������� 
	return ;
}

/**************************************************************************
Function: Analog IIC end signal
Input   : none
Output  : none
�������ܣ�ģ��IIC�����ź�
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C_Stop(void)
{
	SDA_OUT();//sda�����
	IIC_SCL_0;
	IIC_SDA_0;//STOP:when CLK is high DATA change form low to high
 	delay_us(1);
	IIC_SCL_1; 
	IIC_SDA_1;//����I2C���߽����ź�
	delay_us(1);	
}



bool I2C_WaiteForAck(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	IIC_SDA_1;
	delay_us(1);	   
	IIC_SCL_1;
	delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>50)
		{
			I2C_Stop();
			return 0;
		}
	  delay_us(1);
	}
	IIC_SCL_0;//ʱ�����0 	   
	return 1;
}

/**************************************************************************
Function: IIC response
Input   : none
Output  : none
�������ܣ�IICӦ��
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C_Ack(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_0;
	delay_us(1);
	IIC_SCL_1;
	delay_us(1);
	IIC_SCL_0;
}

/**************************************************************************
Function: IIC don't reply
Input   : none
Output  : none
�������ܣ�IIC��Ӧ��
��ڲ�������
����  ֵ����
**************************************************************************/ 
void I2C_NAck(void)
{
	IIC_SCL_0;
	SDA_OUT();
	IIC_SDA_1;
	delay_us(1);
	IIC_SCL_1;
	delay_us(1);
	IIC_SCL_0;
}



bool I2C_WriteOneBit(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitNum, uint8_t Data)
{
    uint8_t Dat;
    
    Dat =I2C_ReadOneByte(DevAddr, RegAddr);
    Dat = (Data != 0) ? (Dat | (1 << BitNum)) : (Dat & ~(1 << BitNum));
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}




bool I2C_WriteBits(uint8_t DevAddr, uint8_t RegAddr, uint8_t BitStart, uint8_t Length, uint8_t Data)
{

    uint8_t Dat, Mask;
    
	Dat = I2C_ReadOneByte(DevAddr, RegAddr);
    Mask = (0xFF << (BitStart + 1)) | 0xFF >> ((8 - BitStart) + Length - 1);
    Data <<= (8 - Length);
    Data >>= (7 - BitStart);
    Dat &= Mask;
    Dat |= Data;
    I2C_WriteOneByte(DevAddr, RegAddr, Dat);
    
    return true;
}

/**************************************************************************
Function: IIC sends a bit
Input   : none
Output  : none
�������ܣ�IIC����һ��λ
��ڲ�������
����  ֵ����
**************************************************************************/
void I2C_WriteByte(uint8_t Data)
{
    uint8_t t;   
	SDA_OUT(); 	    
    IIC_SCL_0;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {        
		if((Data&0x80)>>7 == 0)
		{
			IIC_SDA_0;
		}			
		else
		{
			IIC_SDA_1;
		}
		Data<<=1; 	  
		delay_us(1);   
		IIC_SCL_1;
		delay_us(1); 
		IIC_SCL_0;	
		delay_us(1);
    }	 
}


uint8_t I2C_WriteOneByte(uint8_t DevAddr, uint8_t RegAddr, uint8_t Data)
{
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_WriteByte(Data);
	I2C_WaiteForAck();
	I2C_Stop();
	return 1;
}


bool I2C_WriteBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	
	for(i = 0; i < Num; i ++)
	{
		I2C_WriteByte(*(pBuff + i));
		I2C_WaiteForAck();
	}
	I2C_Stop();

	return true;
}

/**************************************************************************
Function: IIC reads a bit
Input   : none
Output  : none
�������ܣ�IIC��ȡһ��λ
��ڲ�������
����  ֵ����
**************************************************************************/
uint8_t I2C_ReadByte(uint8_t Ack)
{
	uint8_t i, RecDat = 0;

	SDA_IN();
	for(i = 0; i < 8; i ++)
	{
	//	I2C_SCL_Clr();
		IIC_SCL_0;
		delay_us(1);
//		I2C_SCL_Set();
				IIC_SCL_1;
		RecDat <<= 1;
		if(READ_SDA)
			RecDat |= 0x01;
		else
			RecDat &= ~0x01;
		delay_us(1);
	}
	if(I2C_ACK == Ack)
		I2C_Ack();
	else
		I2C_NAck();

	return RecDat;
}




uint8_t I2C_ReadOneByte(uint8_t DevAddr, uint8_t RegAddr)
{
	uint8_t TempVal = 0;
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();
	TempVal = I2C_ReadByte(I2C_NACK);
	I2C_Stop();
	
	return TempVal;
}

bool I2C_ReadBuff(uint8_t DevAddr, uint8_t RegAddr, uint8_t Num, uint8_t *pBuff)
{
	uint8_t i;

	if(0 == Num || NULL == pBuff)
	{
		return false;
	}
	
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Transmitter);
	I2C_WaiteForAck();
	I2C_WriteByte(RegAddr);
	I2C_WaiteForAck();
	I2C_Start();
	I2C_WriteByte(DevAddr | I2C_Direction_Receiver);
	I2C_WaiteForAck();

	for(i = 0; i < Num; i ++)
	{
		if((Num - 1) == i)
		{
			*(pBuff + i) = I2C_ReadByte(I2C_NACK);
		}
		else
		{
			*(pBuff + i) = I2C_ReadByte(I2C_ACK);
		}
	}

	I2C_Stop();
	
	return true;
}


///**************************************************************************
//Function: IIC continuous reading data
//Input   : dev��Target device IIC address��reg:Register address��
//					length��Number of bytes��*data:The pointer where the read data will be stored
//Output  : count��Number of bytes read out-1
//�������ܣ�IIC����������
//��ڲ�����dev��Ŀ���豸IIC��ַ��reg:�Ĵ�����ַ��length���ֽ�����
//					*data:���������ݽ�Ҫ��ŵ�ָ��
//����  ֵ��count�����������ֽ�����-1
//**************************************************************************/ 
//uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data){
//    uint8_t count = 0;
//	
//	IIC_Start();
//	IIC_Send_Byte(dev);	   //����д����
//	IIC_Wait_Ack();
//	IIC_Send_Byte(reg);   //���͵�ַ
//  IIC_Wait_Ack();	  
//	IIC_Start();
//	IIC_Send_Byte(dev+1);  //�������ģʽ	
//	IIC_Wait_Ack();
//	
//    for(count=0;count<length;count++){
//		 
//		 if(count!=length-1)   data[count]=IIC_Read_Byte(1);  //��ACK�Ķ�����
//		 else                  data[count]=IIC_Read_Byte(0);  //���һ���ֽ�NACK
//	}
//    IIC_Stop();//����һ��ֹͣ����
//    return count;
//}



