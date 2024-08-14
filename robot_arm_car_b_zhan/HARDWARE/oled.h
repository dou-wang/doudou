/**
  *****************************************************************************
  * @file			oled.h
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/25
	* @environment	stm32f407
  * @brief 
  * @copyright		HUNAU  
  *****************************************************************************
**/







#ifndef __OLED_H
#define __OLED_H
#include "sys.h"



/*向SSD1106写入一个字节*/	
void OLED_WR_Byte(u8 data,u8 cmd);
/*设置地址*/
void OLED_Set_Pos(u8 x, u8 y);
/*开启OLED显示*/    
void oled_on(void);
/*关闭OLED显示*/     
void oled_Off(void);
/*满屏*/
void oled_full(void);
/*清屏*/
void oled_clear(void); 
/*清页*/
void oled_clear_page(u8 first_page, u8 num);
/*清局部*/
void oled_clear_part(u8 x1, u8 y1, u8 x2, u8 y2);
/*显示字符-16×8*/
void oled_show_char(u8 x,u8 y,u8 _char);
/*显示字符串-16×8*/
void oled_show_string(u8 x,u8 y,u8 *string);
/*数字显示*/
void oled_show_num(u16 x, u16 y, long double num, u8 reserve);
/*中文32*32*/
void oled_show_chinese_32(u8 x,u8 y,u8 numble);
/*图片-128*64*/
void oled_show_picture(u8 *picture);
/*oled初始化*/
void oled_Init(void);
void Oled_Init_Show(void);
void Oled_Refresh(void);




#define 	OLED_CS			GPIO_Pin_6
#define 	OLED_DC			GPIO_Pin_5
#define 	OLED_RES		GPIO_Pin_4
#define 	OLED_SDA		GPIO_Pin_3
#define 	OLED_SCL		GPIO_Pin_2

#define 	OLED_CS_L 		GPIO_ResetBits(GPIOE,OLED_CS)
#define 	OLED_CS_H 		GPIO_SetBits(GPIOE,OLED_CS)

#define 	OLED_DC_L 		GPIO_ResetBits(GPIOE,OLED_DC)
#define 	OLED_DC_H 		GPIO_SetBits(GPIOE,OLED_DC)

#define 	OLED_RES_L 		GPIO_ResetBits(GPIOE,OLED_RES)
#define 	OLED_RES_H 		GPIO_SetBits(GPIOE,OLED_RES)

#define 	OLED_SDA_L 		GPIO_ResetBits(GPIOE,OLED_SDA)
#define 	OLED_SDA_H 		GPIO_SetBits(GPIOE,OLED_SDA)

#define 	OLED_SCL_L 		GPIO_ResetBits(GPIOE,OLED_SCL)
#define 	OLED_SCL_H 		GPIO_SetBits(GPIOE,OLED_SCL)

#define 	OLED_CMD 		0
#define 	OLED_DATA 		1


#endif

/*end of oled.h*/







