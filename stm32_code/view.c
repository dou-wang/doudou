#include "view.h"

uint8_t Page = 0;


void show()
{
	
	switch(Page)
	{
		case 0:	OLED_ShowString(0,0,(uint8_t *)" ",16);
				OLED_ShowNum(8*3,0,100,3,16);

				
				break;	
		
		case 1:	
				break;
		
		default:
			break;
	}
}


