#ifndef __UART5_H
#define __UART5_H

#include "stm32f4xx_usart.h"
#include "stdint.h"
void Uart5Init(unsigned int uiBaud);
void Uart5Send(unsigned char *p_data, unsigned int uiSize);
extern float zangle;
#endif 




