#include "spi.h"

void SPI_W_MOSI(GPIO_PinState GPIO_PIN_X)
{
	
	HAL_GPIO_WritePin(MOSI_GPIO_Port,MOSI_Pin,GPIO_PIN_X);
}

void SPI_W_CS(GPIO_PinState GPIO_PIN_X)
{
	
	HAL_GPIO_WritePin(CS_GPIO_Port,CS_Pin,GPIO_PIN_X);
}

void SPI_W_SCK(GPIO_PinState GPIO_PIN_X)
{
	
	HAL_GPIO_WritePin(SCK_GPIO_Port,SCK_Pin,GPIO_PIN_X);
}

uint8_t SPI_R_MISO(void)
{
	
	return HAL_GPIO_ReadPin(MISO_GPIO_Port,MISO_Pin);
}

void SPI_Init(void)
{
	SPI_W_CS(GPIO_PIN_SET);
	SPI_W_SCK(GPIO_PIN_RESET);
}

void SPI_Start(void)
{
	SPI_W_CS(GPIO_PIN_RESET);
}

void SPI_Stop(void)
{
	SPI_W_CS(GPIO_PIN_SET);
}

uint8_t SPI_Swap_byte(uint8_t SendByte)
{
	uint8_t i,Receive_data;
	
	for(i = 0;i < 8;i++)			//交换数据 （交换字节时序的实现）
	{
		SPI_W_MOSI((GPIO_PinState)(SendByte & (0x80 >> i)));	
		SPI_W_SCK(GPIO_PIN_SET);
		if(SPI_R_MISO() == 1)
		{
			Receive_data |= (0x80 >> i);
		}
		SPI_W_SCK(GPIO_PIN_RESET);
	}
	return Receive_data;
}


