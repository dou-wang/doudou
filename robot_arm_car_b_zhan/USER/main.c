/**
  *****************************************************************************
  * @file			main.c
  * @author			WWJ
  * @version		v1.0
  * @date			2019/4/24
	* @environment	stm32f407
  * @brief   
  * @copyright		HUNAU
  *****************************************************************************
**/



#include "sys.h"
#include "main.h"
char txBuffer[] = "#1GC2\r\n";
//void start_up(void)
//{
//	systick_Init();
//	led_Init();
//	key_Init();
//	oled_Init();
//	motor_Init(MOTOR_ARR,MOTOR_PRE);
//	dj_Init();
//	
//	usart1_Init(115200); /* 蓝牙传输 */
//	usart3_Init(115200); /* 全场定位 */
//	uart4_Init(9600);	 /* 扫 码 器 */
////	uart5_Init(115200);	 /* ESP 8266 */
//	usart6_Init(115200); /* 摄 像 头 */
//	PID_Init();
//	Scheduler_Setup();
//	
//	FLAG.Init_finish = 1;
//}
int x = 10; 
int main(void)
{		systick_Init();
 motor_control_init();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	motor_Init(MOTOR_ARR,MOTOR_PRE);
	encoder_Init();
	usart2_Init(9600);
 TIM9_EnAnWh_Vel();
  motor_on();
	while(1)
	{

	delay_ms(1000);
//		usart3_send(txBuffer, sizeof(txBuffer));
//		Scheduler_Run();
	}
}



#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
	while(1)
	{
		/* printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	}
}
#endif

//void delay_ma(uint32_t time) {
//  uint32_t i;
//  for(i = 0; i < time * 1000; i++);
//}

///* end of main.c */


