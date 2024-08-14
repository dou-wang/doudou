/**
  *****************************************************************************
  * @file			uart.c - 扫码器
  * @author			WWJ
  * @version		v1.0
  * @date			2019/5/15
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/




#include "uart4.h"


void uart4_Init(u32 B)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate   = B;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(UART4, &USART_InitStruct);
	
	USART_ITConfig(UART4,USART_IT_RXNE,ENABLE);
	USART_ITConfig(UART4,USART_IT_TXE,DISABLE); /* 关闭发送中断 */
	
	NVIC_SetPriority(UART4_IRQn,NVIC_EncodePriority(2,1,1));
	NVIC_EnableIRQ(UART4_IRQn);
	
	USART_Cmd(UART4,ENABLE);
}




u8 RX4_Buf[50]  = {0};
u8 TX4_Buf[256] = {0};
u8 TX4_Count = 0;
u8 Count4 = 0;



/* 
	'1':红  
	'2':绿
	'3':蓝		
*/
_CODE Code={0}; 


void uart4_data_analyse(u8 *buf,u8 num)
{
	/* 0-2 */
	Code.first_num = *(buf+0)-48;
	Code.second_num= *(buf+1)-48;
	Code.third_num = *(buf+2)-48;
	
	Code.num = Code.first_num*100 + Code.second_num*10 + Code.third_num; /* 转换为百位数 */

	Code.finish = 1;																																							
}



void uart4_data_receive_prepare(u8 data)
{
	static u8 state = 0;
	
	if(state==0)
	{
		state = 1;
		RX4_Buf[0] = data;
	}
	else if(state==1)
	{
		state = 2;
		RX4_Buf[1] = data;
	}
	else if(state==2)
	{
		state = 3;
		RX4_Buf[2] = data;
		
		if(!Code.finish) /* 如果一次都没扫到 */
		{
			uart4_data_analyse(RX4_Buf,3);
		}
	}
	else
	{
		state = 0;
	}
}


void UART4_IRQHandler(void)
{
	u8 data;
	
	if(USART_GetITStatus(UART4,USART_IT_RXNE))
	{
		USART_ClearITPendingBit(UART4,USART_IT_RXNE);
		
		data = UART4->DR;
		uart4_data_receive_prepare(data);
	}
	
	if(USART_GetITStatus(UART4,USART_IT_TXE))
	{
		UART4->DR = TX4_Buf[TX4_Count++];
		
		if(TX4_Count == Count4)
		{
			UART4->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}


void uart4_send(u8 *data, u8 num)
{
	u8 i;
	
	if(!(UART4->CR1 & USART_CR1_TXEIE))
	{
		for(i=0; i< num; i++)
		{
			TX4_Buf[Count4++] = *(data + i);
		}
		
		USART_ITConfig(UART4, USART_IT_TXE, ENABLE);
	}
}



/* end of uart4.c */


