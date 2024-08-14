/**
  *****************************************************************************
  * @file				usart6.c
  * @author				WWJ
  * @version			v1.0
  * @date				2019/5/15
	* @environment		stm32f407
  * @brief   
  *****************************************************************************
**/



#include "usart6.h"
#include "usart3.h"
#include "ctrl.h"
#include "string.h"
#include "stdlib.h"
char buffer[100];
int bufferIndex = 0;
char t1String[10];
char t2String[10];
float t1, t2;


void usart6_Init(u32 B)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	USART_InitTypeDef USART_InitStruct;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,ENABLE);
	
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource7,GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG,GPIO_PinSource14,GPIO_AF_USART6);
	
	GPIO_InitStruct.GPIO_Pin   = GPIO_Pin_7 | GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode  = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_PuPd  = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOG, &GPIO_InitStruct);
	
	USART_InitStruct.USART_BaudRate   = B;
	USART_InitStruct.USART_Parity     = USART_Parity_No;
	USART_InitStruct.USART_StopBits   = USART_StopBits_1;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(USART6, &USART_InitStruct);
	
	USART_ITConfig(USART6,USART_IT_RXNE,ENABLE);
	USART_ITConfig(USART6,USART_IT_TXE,DISABLE); /* 关闭发送中断 */
	
	NVIC_SetPriority(USART6_IRQn,NVIC_EncodePriority(2,1,1));
	NVIC_EnableIRQ(USART6_IRQn);
	
	USART_Cmd(USART6,ENABLE);
}



u8 RX6_Buf[50]  = {0};
u8 TX6_Buf[256] = {0};
u8 TX6_Count = 0;
u8 Count6 = 0;


_OPENMV Openmv={0};


void usart6_data_analyse(u8 *buf,u8 num)
{
	u8 i,sum=0;
	
	for(i=0; i<(num-1); i++)
	{
		sum += *(buf+i);
	}
	
	if(!(*(buf)==0XAA && *(buf+1)==0XB0) && *(buf+2)==0X0B)
	{
		return;
	}
	
	if(sum == (*(buf+num-1)))
	{
		Openmv.errx = (short)(*(buf+4)<<8 | *(buf+5)); /* 偏右为正 */
		Openmv.erry = ((short)(*(buf+6)<<8 | *(buf+7)))-10; /* 偏下为正 */
		
		/* 未扫到 */
		if(ABS(Openmv.errx) > 250)
		{
			Openmv.errx = 255;
		}
		if(ABS(Openmv.erry) > 250)
		{
			Openmv.erry = 255;
		}
		
		if(*(buf+8) != 0 && Openmv.finish == 0 && X_NOW >= OPENMV_START_DIST)
		{
			if(Openmv.first_num == 0)
			{
				Openmv.first_num = *(buf+8); /* 扫出第一个颜色 */
			}
			if(Openmv.first_num!=0 && *(buf+8)!=Openmv.first_num && Openmv.second_num==0)
			{
				Openmv.second_num = *(buf+8); /* 扫出第二个颜色 */
				
				Openmv.third_num = 6 - (Openmv.first_num+Openmv.second_num); /* 确定第三个颜色 */
				
				Openmv.num = Openmv.first_num*100 + Openmv.second_num*10 + Openmv.third_num; /* 转换为百位数 */	
				
				Openmv.finish = 1;
				
				/* 确定物料区从左到右扫到颜色的值 */
				COM_Tasks[0].color_num = Openmv.first_num;
				COM_Tasks[1].color_num = Openmv.second_num;
				COM_Tasks[2].color_num = Openmv.third_num;
				
				/*----------------------------123-------------------------------*/
				if(ZONE1 == 123)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:红/绿/蓝 */								 
						case 1:  COM_Tasks[0].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[0].put[X] = THIRD_X_PUT ;break;
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[1].put[X] = THIRD_X_PUT ;break;
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[2].put[X] = THIRD_X_PUT ;break;
						default:break;
					}
				}
				
				
				/*----------------------------132-------------------------------*/
				if(ZONE1 == 132)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:红/蓝/绿 */	
						case 1:  COM_Tasks[0].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[0].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[1].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = FIRST_X_PUT ;break;
						case 2:  COM_Tasks[2].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						default:break;
					}
				}
				
				
				/*----------------------------213-------------------------------*/
				if(ZONE1 == 213)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:绿/红/蓝 */								 
						case 1:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[0].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[0].put[X] = THIRD_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[1].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[1].put[X] = THIRD_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[2].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[2].put[X] = THIRD_X_PUT;break; 
						default:break;
					}
				}
				
				/*----------------------------231-------------------------------*/
				if(ZONE1 == 231)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:绿/蓝/红 */								 
						case 1:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[0].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[0].put[X] = FIRST_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[1].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[1].put[X] = FIRST_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						case 2:  COM_Tasks[2].put[X] = THIRD_X_PUT;break; 
						case 3:  COM_Tasks[2].put[X] = FIRST_X_PUT;break; 
						default:break;
					}
				}
				
				/*----------------------------312-------------------------------*/
				if(ZONE1 == 312)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:蓝/红/绿 */								 
						case 1:  COM_Tasks[0].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[0].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[1].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[2].put[X] = FIRST_X_PUT;break; 
						case 3:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						default:break;
					}
				}
				
				/*----------------------------321-------------------------------*/
				if(ZONE1 == 321)
				{
					/* 确定第一次存放时相应颜色对应位置 */
					switch (COM_Tasks[0].color_num)
					{	/* 放置区1顺序:蓝/绿/红 */								 
						case 1:  COM_Tasks[0].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[0].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[0].put[X] = FIRST_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[1].color_num)              
					{
						case 1:  COM_Tasks[1].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[1].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[1].put[X] = FIRST_X_PUT;break; 
						default:break;
					}                                       
					switch (COM_Tasks[2].color_num)
					{                                       
						case 1:  COM_Tasks[2].put[X] = THIRD_X_PUT;break; 
						case 2:  COM_Tasks[2].put[X] = SECOND_X_PUT;break;
						case 3:  COM_Tasks[2].put[X] = FIRST_X_PUT;break; 
						default:break;
					}
				}
			}
		}
	}
}


void usart6_data_receive_prepare(char ch)
{  char *tok;
	 buffer[bufferIndex] = ch;
        bufferIndex++;

		if (ch == '\n')
		{
        	buffer[bufferIndex] = '\0';
        	tok = strtok(buffer, " ");
        	while (tok != NULL)
        	{
            	if (strncmp(tok, "t1=", 3) == 0)
            	{
                	strcpy(t1String, tok + 3);
                	t1 = atof(t1String);
            	}
            	else if (strncmp(tok, "t2=", 3) == 0)
            	{
                	strcpy(t2String, tok + 3);
                	t2 = atof(t2String);
            	}
            	tok = strtok(NULL, " ");
        	}
	        bufferIndex = 0;
    	}
}
		

void USART6_IRQHandler(void)
{
	char ch;
	
	if(USART_GetITStatus(USART6,USART_IT_RXNE))
	{
		USART_ClearITPendingBit(USART6,USART_IT_RXNE);
		
		ch = USART6->DR;
		usart6_data_receive_prepare(ch);
	}
	
	if(USART_GetITStatus(USART6,USART_IT_TXE))
	{
		USART6->DR = TX6_Buf[TX6_Count++];
		
		if(TX6_Count == Count6)
		{
			USART6->CR1 &= ~USART_CR1_TXEIE;
		}
	}
}


void usart6_send(u8 *data, u8 num)
{
	u8 i;
	
	if(!(USART6->CR1 & USART_CR1_TXEIE))
	{
		for(i=0; i< num; i++)
		{
			TX6_Buf[Count6++] = *(data + i);
		}
		
		USART_ITConfig(USART6, USART_IT_TXE, ENABLE);
	}
}



/* end of usart6.c */


