#include "delay.h"
int fac_us=0;							//us延时倍乘数
extern uint32_t HAL_RCC_GetHCLKFreq(void);		 
//初始化延迟函数
void delay_init()
{
	fac_us=HAL_RCC_GetHCLKFreq()/1000000;
}								 	

void delay_us(int nus)
{		
	int ticks,told,tnow,tcnt=0;
	int reload=SysTick->LOAD;				
	//LOAD的值	    	 
	ticks=nus*fac_us; 						
	//需要的节拍数 
	told=SysTick->VAL;        				
	//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			
		}  
	};
}
//延时nms
//nms:要延时的ms数
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}
