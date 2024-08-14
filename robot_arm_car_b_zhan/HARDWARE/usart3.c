/**
  *****************************************************************************
  * @file				usart3.c
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/7
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/




#include "usart3.h"




void usart3_Init(u32 B)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate   = B;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART3, &USART_InitStruct);
	
	USART_ITConfig(USART3,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART3,USART_IT_TXE,DISABLE); /* 关闭发送中断 */
	
	NVIC_SetPriority(USART3_IRQn,NVIC_EncodePriority(2,1,1));
	NVIC_EnableIRQ(USART3_IRQn);
	
	USART_Cmd(USART3,ENABLE);
}



u8 RX3_Buf[50]  = {0};
u8 TX3_Buf[256] = {0};
u8 TX3_Count = 0;
u8 Count3 = 0;

float pos_x  = 0.0f;
float pos_y  = 0.0f;
float xangle = 0.0f;
float yangle = 0.0f;
float w_z    = 0.0f;


static union
{
	u8 data[24];
	float Val[6];
	
}posture;




void usart3_data_receive_prepare(u8 data)
{
	static u8 data_len = 0,R_num = 0,i;
	static u8 state = 0;
	
	if(state==0 && data==0X0D)
	{
		state=1;
		R_num = 0;
		RX3_Buf[R_num++] = data; /* 1 */
	}
	else if(state==1 && data==0X0A) 
	{
		state=2;
		i = 0;
		data_len = 24;
		RX3_Buf[R_num++] = data; /* 2 */
	}
	else if(state==2 && data_len>0)
	{
		data_len--;
		posture.data[i++] = data;
		RX3_Buf[R_num++] = data; /*3-26*/
		
		if(data_len == 0)
		{
			state = 3;
		}
	}
	else if(state == 3 && data == 0X0A) /* 0X0A */
	{
		state = 4;
		RX3_Buf[R_num++] = data; /* 27 */
	}
	else if(state == 4 && data == 0X0D) /* 0X0D */
	{
		state = 0;
		RX3_Buf[R_num] = data; /* 28 */
		
		zangle = posture.Val[0];
		xangle = posture.Val[1];
		yangle = posture.Val[2];
		pos_x  = posture.Val[3];
		pos_y  = posture.Val[4];
		w_z    = posture.Val[5];
		
		if(zangle < -135)
		{
			zangle = 360 + zangle;
		}
	}
	else
	{
		state = 0;
	}
}



void USART3_IRQHandler(void)
{
	u8 data;
	
	if(USART_GetITStatus(USART3,USART_IT_TXE))
	{
		USART3->DR = TX3_Buf[TX3_Count++];
		
		if(TX3_Count == Count3)
		{
			USART3->CR1 &= ~USART_CR1_TXEIE;
		}
	}
	
	if(USART_GetITStatus(USART3, USART_IT_RXNE))
	{
		USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		
		data = USART_ReceiveData(USART3);
		
		usart3_data_receive_prepare(data);
	}
}




/* x字节+num字节字符串组合 */
void Strcat(char str1[],char str2[],u8 num)
{
	int i=0,j=0;

	while(str1[i]!='\0')
	i++;
	
	for(j=0;j<num;j++)
	{
		str1[i++]=str2[j];
	}
}


/* 串口发送 */
void usart3_send(char *data, u8 num)
{
	u8 i;
	
	if(!(USART3->CR1 & USART_CR1_TXEIE))
	{
		for(i=0; i< num; i++)
		{
			TX3_Buf[Count3++] = *(data + i);
		}
		
		USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
	}
}

void usart3_send1(char *data, u8 num, u8 x)
{
    u8 i;
    char buffer[50];

    if (!(USART3->CR1 & USART_CR1_TXEIE))
    {
        sprintf(buffer, "距离的值为t=%d", x);
        
        for (i = 0; i < strlen(buffer); i++)
        {
            TX3_Buf[Count3++] = buffer[i];
        }
        
        USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
    }
}

/* 清零 */
void Zero_Clearing(void)
{
	char zero_clear[4] = "ACT0";
	usart3_send(zero_clear,sizeof(zero_clear));
}


/* 更新yaw角 - -180~180 */
void Update_A(float angle)
{
	char update_yaw[8] = "ACTJ";
	static union
	{
		float A;
		char data[4];
		
	}set;
	
	set.A = angle;
	
	Strcat(update_yaw,set.data,4);
	
	usart3_send(update_yaw,sizeof(update_yaw));
}


/* 更新X */
void Update_X(float posx)
{
	char update_x[8] = "ACTX";
	static union
	{
		float X;
		char data[4];
		
	}set;
	
	set.X = posx;
	
	Strcat(update_x,set.data,4);
	
	usart3_send(update_x,sizeof(update_x));
}


/* 更新Y */
void Update_Y(float posy)
{
	char update_y[8] = "ACTY";
	static union
	{
		float Y;
		char data[4];
		
	}set;
	
	set.Y = posy;
	
	Strcat(update_y,set.data,4);
	
	usart3_send(update_y,sizeof(update_y));
}


/* 更新XY */
void Update_XY(float posx,float posy)
{
	char update_xy[12] = "ACTD";
	static union
	{
		float XY[2];
		char data[8];
		
	}set;
	
	set.XY[0] = posx;
	set.XY[1] = posy;
	
	Strcat(update_xy,set.data,8);
	
	usart3_send(update_xy,sizeof(update_xy));
}




/* end of usart3.c */


