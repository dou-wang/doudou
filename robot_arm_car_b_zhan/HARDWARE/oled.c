/**
  *****************************************************************************
  * @file			oled.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/






#include "oled.h"
#include "oled_font.h"
#include "main.h"
																											



/*向SSD1106写入一个字节*/	
void OLED_WR_Byte(u8 data,u8 cmd)
{	
	u8 i;			  
	if(cmd)	/*数据*/
	{
		OLED_DC_H;
	}
	else	/*命令*/
	{		
		OLED_DC_L;		  
	}
	
	OLED_CS_L;	/*片选拉低*/
	
	for(i=0;i<8;i++)
	{			  
		OLED_SCL_L;
		if(data&0x80)
		{
			OLED_SDA_H;
		}
		else
		{			
			OLED_SDA_L;
		}
		OLED_SCL_H;
		data<<=1;   
	}
	
	OLED_CS_H;
	OLED_DC_H;  
}





/*设置地址*/
void OLED_Set_Pos(u8 x, u8 y) 
{ 
	OLED_WR_Byte(0xb0+y,OLED_CMD);      		/*设置页地址(0~7)*/
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);  /*设置显示位置—列高地址*/   
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);   	/*设置显示位置—列低地址*/
}  





/*开启OLED显示*/    
void oled_on(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  /*SET DCDC命令*/
	OLED_WR_Byte(0X14,OLED_CMD);  /*DCDC ON*/
	OLED_WR_Byte(0XAF,OLED_CMD);  /*DISPLAY ON*/
}





/*关闭OLED显示*/     
void oled_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  /*SET DCDC命令*/
	OLED_WR_Byte(0X10,OLED_CMD);  /*DCDC OFF*/
	OLED_WR_Byte(0XAE,OLED_CMD);  /*DISPLAY OFF*/
}	





/*满屏*/
void oled_full(void)
{
	u8 i,j;
	for(i=0; i<8; i++)
	{
		OLED_Set_Pos(0,0+i);
		for(j=0;j<127;j++)	/*x地址自增*/
		{ 
			OLED_WR_Byte(0xff,OLED_DATA); /*0xff代表点亮该列8行*/
			#if 0	
				delay_ms(500);	/*通过延时查看显示过程*/
			#endif
		}
	}
}





/*清屏*/
void oled_clear(void)  
{ 
	u8 i,j;	
	for(i=0; i<8; i++)  
	{  
		OLED_WR_Byte(0xb0+i,OLED_CMD);	/*设置页地址(0~7)*/
		OLED_WR_Byte(0x00,OLED_CMD);  	/*设置显示位置—列低地址*/
		OLED_WR_Byte(0x10,OLED_CMD);  	/*设置显示位置—列高地址*/ 
		for(j=0; j<128; j++)
		{
			OLED_WR_Byte(0,OLED_DATA); 	/*写0清屏*/
		}
	}
}





/*清页*/
void oled_clear_page(u8 first_page, u8 num)
{
	u8 i,j;
	
	for(i=first_page; i<first_page+num; i++)
	{
		OLED_Set_Pos(0,i);
		for(j=0; j<128; j++)
		{
			OLED_WR_Byte(0,OLED_DATA);
		}
	}
}





/*清局部*/
void oled_clear_part(u8 x1, u8 y1, u8 x2, u8 y2)
{
	u8 i,j,x0,y0,temp;
	
	if(x1>x2)
	{
		temp = x2;
		x2 = x1;
		x1 = temp;
	}
	if(y1>y2)
	{
		temp = y2;
		y2 = y1;
		y1 = temp;
	}
	
	x0=x2-x1;
	y0=y2-y1;
	
	for(i=0; i<y0; i++)
	{
		for(j=0; j<x0; j++)
		{
			OLED_Set_Pos(x1+j,y1+i);
			for(j=0; j<x0; j++)	/*x地址自增*/
			{
				OLED_WR_Byte(0x00,OLED_DATA);
				#if 0	
					delay_ms(500);	/*通过延时查看显示过程*/
				#endif
			}			
		}
	}
}





/*显示字符-16×8*/
void oled_show_char(u8 x,u8 y,u8 _char)
{
	u8 i,c;
	c=_char-' '; /*得到ASCII码*/
	
	OLED_Set_Pos(x,y); /*第一页*/
	for(i=0; i<8; i++)
	{
		#ifdef DEBUG
		delay_ms(1000);
		#endif
		OLED_WR_Byte(ASCII_16_8[c*16+i],OLED_DATA); /*扫描前8位*/
	}
	
	OLED_Set_Pos(x,y+1); /*第二页*/
	for(i=0; i<8; i++)		
	{	
		#ifdef DEBUG
		delay_ms(1000);
		#endif
		OLED_WR_Byte(ASCII_16_8[c*16+i+8],OLED_DATA); /*扫描后8位*/
	}
}



/*显示字符-32×16*/
void oled_show_num_0_9_32_32(u8 x,u8 y,u8 num)
{
	u8 i,j;
	
//	#define DEBUG
	
	for(j = 0 ; j < 4 ; j ++)
	{
		OLED_Set_Pos(x,y+j); /*第一页*/
		for(i=0; i<16; i++)
		{
			#ifdef DEBUG
				delay_ms(50);
			#endif
			OLED_WR_Byte(ASCII_32_16[num*4+j][i],OLED_DATA);
		}
	}
}




/*显示字符串-16×8*/
#define LINE_FEED	0 /* 1为自动换行,0为不换行 */
void oled_show_string(u8 x,u8 y,u8 *string)
{
	u8 j=0;
	while(string[j] != '\0') /*字符串最后玩为结束符'\0'*/
	{
		oled_show_char(x,y,string[j]);
		x+=8; /*行排列*/
		j++;
		
		if(x>120)
		{ 
			if(!LINE_FEED)
				break;
			else
			{
				x=0;
				y+=2;
			}
		}
	}
}





/*数字显示*/
void oled_show_num(u16 x, u16 y, long double num, u8 reserve)
{
	u8 temp[20]={0},i=0,temp_i,temp1[10]={0},j=0,temp_j,k;	
	long double temp_num;
	long znum,xnum,znum_temp;
	
	temp_num = num; /*保存数字*/
	if(num<0) /*负数取反*/
	{
		num = 0-num;
	}
	if(reserve>9)/*保留位数上限*/
	{
		reserve = 9;
	}
	znum = (u32)num; /*获取整数部分*/
	znum_temp = znum;
	xnum = (num-znum)*pow(10,reserve); /*获取小数部分,显示误差来源于此*/
	do /*整数部分存入数组*/
	{
		temp1[j++] = znum%10;
		znum = znum/10;
	}while(znum>0);
	
	if(temp_num<0) /*负数添加负号*/
	{
		temp1[j++] = (char)('-'-48);
	}
	
	temp_j = j;
	j = j-1;
	
	do /*调转方向*/
	{
		temp[i++] = temp1[j--]+48; /*'0'的ASCII为48*/
	}while(i<temp_j);
	
	if(reserve != 0)
	{
		j = 0; /*temp1数组清空*/

		if(xnum == 0)
		{
			for(k=0; k<reserve; k++)
			{
				temp1[j++] = 0;
			}
		}
		else
		{ 
			do /*小数部分存入数组*/
			{
				temp1[j++] = xnum%10;
				xnum = xnum/10;
			}while(xnum>0);
			
			k = 1;
			
			while(k)
			{
				if((u16)((num-znum_temp)*pow(10,k)) == 0)
				{
					k++;
					temp1[j++] = 0;
				}
				else
				{
					k = 0;
				}
			}
		}
		temp1[j++] = (char)('.'-48); /*存入小数点*/
		
		temp_j = j;
		temp_i = i+temp_j;
		j=j-1;
		
		do /*调转方向*/
		{
			temp[i++] = temp1[j--]+48;
		}while(i<temp_i);
	}

	oled_show_string(x,y,temp); /*显示*/
}





/*中文32*32*/
void oled_show_chinese_32(u8 x,u8 y,u8 numble)
{
	u8 i,j;
	for(i=0; i<4; i++)
	{
		OLED_Set_Pos(x,y+i);
		for(j=0; j<32; j++)
		{
			OLED_WR_Byte(CH_32[4*numble+i][j],OLED_DATA);
			#if 0
				delay_ms(50);
			#endif
		}
	}
}





/*图片-128*64*/
void oled_show_picture(u8 *picture)
{
	u8 i,j;
	for(i=0; i<8; i++)
	{
		OLED_Set_Pos(0,i);
		for(j=0; j<128; j++)
		{
			OLED_WR_Byte(*picture,OLED_DATA);
			picture++;
			#if 0
				delay_ms(10);
			#endif
		}
	}
}





/*oled初始化*/
void oled_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,ENABLE);
	
	GPIO_InitStruct.GPIO_Pin=OLED_CS|OLED_DC|OLED_RES|OLED_SDA|OLED_SCL;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;
	GPIO_Init(GPIOE,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOE,OLED_CS|OLED_DC|OLED_RES|OLED_SDA|OLED_SCL);
	
	OLED_RES_H;
	delay_ms(100);
	OLED_RES_L;
	delay_ms(100);
	OLED_RES_H;
	delay_ms(100);
	
	OLED_WR_Byte(0xAE,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0x10,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x81,OLED_CMD);
	OLED_WR_Byte(0xCF,OLED_CMD);
	OLED_WR_Byte(0xA0,OLED_CMD); /*0xA0左右反置 0xA1正常*/
	OLED_WR_Byte(0xC0,OLED_CMD); /*0xC0上下反置 0xC8正常*/
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_WR_Byte(0xA8,OLED_CMD);
	OLED_WR_Byte(0x3f,OLED_CMD);
	OLED_WR_Byte(0xD3,OLED_CMD);
	OLED_WR_Byte(0x00,OLED_CMD);
	OLED_WR_Byte(0xd5,OLED_CMD);
	OLED_WR_Byte(0x80,OLED_CMD);
	OLED_WR_Byte(0xD9,OLED_CMD);
	OLED_WR_Byte(0xF1,OLED_CMD);
	OLED_WR_Byte(0xDA,OLED_CMD);
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);
	OLED_WR_Byte(0x40,OLED_CMD);
	OLED_WR_Byte(0x20,OLED_CMD);
	OLED_WR_Byte(0x02,OLED_CMD);
	OLED_WR_Byte(0x8D,OLED_CMD);
	OLED_WR_Byte(0x14,OLED_CMD);
	OLED_WR_Byte(0xA4,OLED_CMD);
	OLED_WR_Byte(0xA6,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_WR_Byte(0xAF,OLED_CMD);
	oled_clear();
	OLED_Set_Pos(0,0);
	  
	Oled_Init_Show();
}




void Oled_Init_Show(void)
{
	oled_show_string(64,0,(u8*)"X:");
	oled_show_string(64,2,(u8*)"Y:");
	oled_show_string(64,4,(u8*)"A:");
}



extern int Tcnt;
extern u16 time;
void Oled_Refresh(void)
{ 
	static u8 remove = 0;
	
	if(FLAG.Task_start == 1)
	{
		if(remove == 0)
		{
			remove = 1;
			oled_clear();
		}
		oled_show_num_0_9_32_32(15,0,Code.first_num);
		oled_show_num_0_9_32_32(55,0,Code.second_num);
		oled_show_num_0_9_32_32(95,0,Code.third_num);
	}
	else
	{
		oled_show_num(80,0,X_NOW,5);
		oled_show_num(80,2,Y_NOW,5);
		oled_show_num(80,4,A_NOW,5);

		oled_show_num(0,0,Openmv.errx,1);
		oled_show_num(0,2,Openmv.erry,1);
		oled_show_num(0,4,Openmv.num,1);
		oled_show_num(0,6,time,1);
		oled_show_num(50,6,YPID.I_OUT,1);
	}
}



/*end of oled.c*/


