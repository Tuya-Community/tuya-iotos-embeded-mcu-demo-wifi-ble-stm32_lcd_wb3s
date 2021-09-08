#include "delay.h"
int fac_us=0;							//us��ʱ������
extern uint32_t HAL_RCC_GetHCLKFreq(void);		 
//��ʼ���ӳٺ���
void delay_init()
{
	fac_us=HAL_RCC_GetHCLKFreq()/1000000;
}								 	

void delay_us(int nus)
{		
	int ticks,told,tnow,tcnt=0;
	int reload=SysTick->LOAD;				
	//LOAD��ֵ	    	 
	ticks=nus*fac_us; 						
	//��Ҫ�Ľ����� 
	told=SysTick->VAL;        				
	//�ս���ʱ�ļ�����ֵ
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
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(uint16_t nms)
{
	uint32_t i;
	for(i=0;i<nms;i++) delay_us(1000);
}
