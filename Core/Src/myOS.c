#include "myOS.h"
#include "stm32l476xx.h"
#include <stdio.h>
#include <main.h>
#include <delay.h>
#include "wifi.h"

__IO unsigned long int system_running_timer = 0U;

//系统管理函数
void System_Management(void)
{
	static __IO uint8_t timer_50ms = 0U;
	static __IO uint8_t timer_500ms = 0U;
	static __IO uint8_t timer_1000ms = 0U;
	static __IO uint8_t timer_10s = 0U;

	system_running_timer++;
	System_Run_2ms();		/*2ms task*/
	timer_50ms++;
	/*write if-else to aviod multiple task are running at same time
	only running 2ms task and one of 50ms, 500ms and 1s every 2ms.*/
	if(timer_50ms >= 25u)
	{
		timer_50ms = 0u;
		Systen_Run_50ms();
		timer_500ms ++;
	}
	else if(timer_500ms >= 10u)
	{
		timer_500ms = 0u;
		System_Run_500ms();	
		timer_1000ms ++;
	}
	else if(timer_1000ms >= 2u)
	{
		timer_1000ms = 0u;
		System_Run_1000ms();
		timer_10s ++;
	}
	else
	{
	
	}
}

/*
	2ms task
*/
void System_Run_2ms(void)
{
	system_running_timer += 2;  /*Record system running time*/	
	
}

/*
	50ms task
*/
void Systen_Run_50ms(void)
{ 
//  Deal_Data_Display();			
}

/*
	500ms task
*/
void System_Run_500ms(void)
{
	//可以在此处加要处理的任务
}

/*
	1s task
*/
void System_Run_1000ms(void)
{
	//可以在此处加要处理的任务
}



