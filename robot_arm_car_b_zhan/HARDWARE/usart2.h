// usart.h

#ifndef USART_H
#define USART_H

#include "stm32f4xx.h"
#include <stdio.h>
int fputc(int ch, FILE *file);
void usart2_Init(uint32_t baud_rate);
int usart2_SendChar(int ch);

#endif // USART_H




