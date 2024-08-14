/**
  *****************************************************************************
  * @file				usart1.c - 适用于QF串口助手
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/8
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#include "usart1.h"
#include "main.h"


#define BYTE0(dwTemp)       *((char *)(&dwTemp)    )
#define BYTE1(dwTemp)       *((char *)(&dwTemp) + 1)
#define BYTE2(dwTemp)       *((char *)(&dwTemp) + 2)
#define BYTE3(dwTemp)       *((char *)(&dwTemp) + 3)
	


void usart1_Init(u32 B)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate   = B;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART1, &USART_InitStruct);
	
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART1,USART_IT_TXE,DISABLE); /* 关闭发送中断 */
	
	NVIC_SetPriority(USART1_IRQn,NVIC_EncodePriority(2,1,1));
	NVIC_EnableIRQ(USART1_IRQn);
	
	USART_Cmd(USART1,ENABLE);
}



u8 RX1_Buf[200] = {0};
u8 TX1_Buf[200] = {0};
u8 TX1_Count = 0;
u8 Count1 = 0;

_RX rx = {0};


void receive_analyse(u8 data[],u8 num)
{
	u8 i, sum=0;
	
	for (i = 0; i < (num - 1); i++)
	{
		sum += data[i];
	}
	
	if (!(data[0] == 0XA5 && data[1] == 0X5A)) /* 检测帧头 */
	{
		return;
	}
	
	if(sum == *(data+num-1))
	{
		switch(*(data+2))
		{
			case 0XF1:
					PID1_P = (float)(*(data+4) << 24 | *(data+5)  << 16 | *(data+6) << 8 | *(data+7) << 0) / 1000;
					PID1_I = (float)(*(data+8) << 24 | *(data+9)  << 16 | *(data+10)<< 8 | *(data+11)<< 0) / 1000;
					PID1_D = (float)(*(data+12)<< 24 | *(data+13) << 16 | *(data+14)<< 8 | *(data+15)<< 0) / 1000;
																				
					PID2_P = (float)(*(data+16) << 24 | *(data+17) << 16 | *(data+18) << 8 | *(data+19) << 0) / 1000;
					PID2_I = (float)(*(data+20) << 24 | *(data+21) << 16 | *(data+22) << 8 | *(data+23) << 0) / 1000;
					PID2_D = (float)(*(data+24) << 24 | *(data+25) << 16 | *(data+26) << 8 | *(data+27) << 0) / 1000;
																				  
					PID3_P = (float)(*(data+28) << 24 | *(data+29) << 16 | *(data+30) << 8 | *(data+31) << 0) / 1000;
					PID3_I = (float)(*(data+32) << 24 | *(data+33) << 16 | *(data+34) << 8 | *(data+35) << 0) / 1000;
					PID3_D = (float)(*(data+36) << 24 | *(data+37) << 16 | *(data+38) << 8 | *(data+39) << 0) / 1000;
					break;
					
			case 0XF2:
					rx.send_pid = 1;
					break;
			case 0XF3:
					rx.dir_up 	= *(data+4)>>0 & 0X01;
					rx.dir_dowm = *(data+4)>>1 & 0x01;
					rx.dir_left = *(data+4)>>2 & 0x01;
					rx.dir_right= *(data+4)>>3 & 0x01;
					break;
		}
	}
}


void receive_prepare(u8 data)
{
	static u8 cnt,data_length;
	static u8 step = 0;
	
	if (step == 0 && data == 0xA5)
	{
		cnt = 0;
		step = 1;
		RX1_Buf[cnt++] = data;
	}
	else if (step == 1 && data == 0x5A)
	{
		step = 2;
		RX1_Buf[cnt++] = data;
	}
	else if (step == 2) /* 功能字 */
	{
		step = 3;
		RX1_Buf[cnt++] = data;
	}
	else if (step == 3 && data < 95)
	{
		step = 4;
		data_length = data;
		RX1_Buf[cnt++] = data;
	}
	else if (step == 4 && data_length > 0)
	{
		data_length--;
		RX1_Buf[cnt++] = data;
		
		if (data_length == 0)
		{
			step = 5;
		}
	}
	else if (step == 5)
	{
		step = 0;
		RX1_Buf[cnt++] = data;
		receive_analyse(RX1_Buf, cnt);
	}

	else
	{
		step = 0;
	}
}




void USART1_IRQHandler(void)
{
	u8 data;
	
	if(USART_GetITStatus(USART1,USART_IT_RXNE))
	{
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		
		data = USART1->DR;
		receive_prepare(data);
	}
	
	if(USART_GetITStatus(USART1,USART_IT_TXE))
	{
		USART1->DR = TX1_Buf[TX1_Count++];
		
		if(TX1_Count == Count1)
		{
			Count1 = 0;
			TX1_Count = 0;
			USART1->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}



void usart1_send(u8 *data, u8 num)
{
	u8 i;
	
	if(!(USART1->CR1 & USART_CR1_TXEIE))
	{
		for(i=0; i< num; i++)
		{
			TX1_Buf[Count1++] = *(data + i);
		}
		
		USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
	}
}



void send_PID(void)
{
	u8 i, k = 0;
	u32 sum = 0;
	int temp[9];
	static u8 send_buf[100];

	temp[0] = PID1_P*1000;
	temp[1] = PID1_I*1000;
	temp[2] = PID1_D*1000;
	
	temp[3] = PID2_P*1000;
	temp[4] = PID2_I*1000;
	temp[5] = PID2_D*1000;
	
	temp[6] = PID3_P*1000;
	temp[7] = PID3_I*1000;
	temp[8] = PID3_D*1000;
	
	send_buf[k++] = 0XA5;
	send_buf[k++] = 0X5A;
	send_buf[k++] = 0XF1; /* 功能字 */

	send_buf[k++] = 0;
	
	//PID1
	send_buf[k++] = BYTE3(temp[0]);
	send_buf[k++] = BYTE2(temp[0]);
	send_buf[k++] = BYTE1(temp[0]);
	send_buf[k++] = BYTE0(temp[0]);

	send_buf[k++] = BYTE3(temp[1]);
	send_buf[k++] = BYTE2(temp[1]);
	send_buf[k++] = BYTE1(temp[1]);
	send_buf[k++] = BYTE0(temp[1]);

	send_buf[k++] = BYTE3(temp[2]);
	send_buf[k++] = BYTE2(temp[2]);
	send_buf[k++] = BYTE1(temp[2]);
	send_buf[k++] = BYTE0(temp[2]);
	
	//PID2
	send_buf[k++] = BYTE3(temp[3]);
	send_buf[k++] = BYTE2(temp[3]);
	send_buf[k++] = BYTE1(temp[3]);
	send_buf[k++] = BYTE0(temp[3]);
                             
	send_buf[k++] = BYTE3(temp[4]);
	send_buf[k++] = BYTE2(temp[4]);
	send_buf[k++] = BYTE1(temp[4]);
	send_buf[k++] = BYTE0(temp[4]);
                             
	send_buf[k++] = BYTE3(temp[5]);
	send_buf[k++] = BYTE2(temp[5]);
	send_buf[k++] = BYTE1(temp[5]);
	send_buf[k++] = BYTE0(temp[5]);
	
	//PID3
	send_buf[k++] = BYTE3(temp[6]);
	send_buf[k++] = BYTE2(temp[6]);
	send_buf[k++] = BYTE1(temp[6]);
	send_buf[k++] = BYTE0(temp[6]);
                             
	send_buf[k++] = BYTE3(temp[7]);
	send_buf[k++] = BYTE2(temp[7]);
	send_buf[k++] = BYTE1(temp[7]);
	send_buf[k++] = BYTE0(temp[7]);
                             
	send_buf[k++] = BYTE3(temp[8]);
	send_buf[k++] = BYTE2(temp[8]);
	send_buf[k++] = BYTE1(temp[8]);
	send_buf[k++] = BYTE0(temp[8]);
	
	send_buf[3] = k - 4;
	
	for(i=0; i<k; i++)
	{
		sum += send_buf[i];
	}

	send_buf[k++] = sum;

	usart1_send(send_buf, k);
	
	k = 0;
}


void usart1_Task(void)
{
	if(rx.send_pid)
	{
		send_PID();
		rx.send_pid = 0;
	}
}



/* end of usart1.h */


