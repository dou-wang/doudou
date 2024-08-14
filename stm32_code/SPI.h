#ifndef __SPI_H
#define __SPI_H

#include "mytask.h"


void SPI_W_MOSI(GPIO_PinState GPIO_PIN_X);

void SPI_W_CS(GPIO_PinState GPIO_PIN_X);

void SPI_W_SCK(GPIO_PinState GPIO_PIN_X);

uint8_t SPI_R_MISO(void);

void SPI_Init(void);
	
void SPI_Start(void);

void SPI_Stop(void);

uint8_t SPI_Swap_byte(uint8_t SendByte);
#endif

