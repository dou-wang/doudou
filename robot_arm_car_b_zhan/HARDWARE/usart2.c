/**
  *****************************************************************************
  * @file				usart2.c
  * @author				WWJ
  * @version			v1.0
  * @date				2019/4/25
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#include "usart2.h"



int fputc(int ch, FILE *file)
{
	while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
	USART_SendData(USART2, ch);
	return ch;
}

void usart2_Init(u32 B)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate   = B;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART2, &USART_InitStruct);
	
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART2,USART_IT_TXE,DISABLE); /* 关闭发送中断 */
	
	NVIC_SetPriority(USART2_IRQn,NVIC_EncodePriority(2,1,1));
	NVIC_EnableIRQ(USART2_IRQn);
	
	USART_Cmd(USART2,ENABLE);
}



/**
  *****************************************************************************
  * @file				usart2.c
  * @author				WWJ
  * @version			v1.0
  * @date				2019/4/25
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



// usart.c





int usart2_SendChar(int ch)
{
    // 发送一个字符到串口
    USART_SendData(USART2, (uint8_t)ch);

    // 等待发送完毕
    while (!(USART2->SR & USART_SR_TXE));

    return ch;
}



/* end of usart2.c */
