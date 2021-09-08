/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************/
#include "main.h"
#include "stm32l4xx_it.h"
#include "mcu_api.h"

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim3;

uint8_t usart1_rxBuf[50];
unsigned char Display_Flag=0; 


/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
 
  while (1)
  {
  }
 
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  
  while (1)
  {
 
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  
  while (1)
  {
  
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
 
  while (1)
  {
   
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
 
  while (1)
  {
    
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{

}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
 
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
 
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();

}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
	uint8_t usart1_data;
	uint8_t send[2]={0x5a,0xa5};
	uint16_t addr=0,data=0;//定义地址及数据
	//uint16_t len=0;		//定义长度 
  uint8_t usart1_counter=0;
  if((USART1->ISR & 1<<5) == 1<<5)//接收寄存器数据不为空
	{
		usart1_data=USART1->RDR;
	usart1_rxBuf[usart1_counter] = usart1_data;	
	if(usart1_counter < 2)
	{	
		if(usart1_rxBuf[usart1_counter]==send[usart1_counter])
		{
			usart1_counter++;
		}
		else
		{
			usart1_counter=0;
		}
	}
	else
	{
		usart1_counter++;
	}
	if(usart1_counter>=(usart1_rxBuf[2]+3))
	{
			addr = usart1_rxBuf[4]*256+usart1_rxBuf[5];				//获取lcd控件寄存器地址
			//len =  usart1_rxBuf[6];													//获取接收的数据字个数
			data = usart1_rxBuf[7]*256+usart1_rxBuf[8];				//获取第一个字数据的值
		switch(addr)
		 {
			 case 0x1000:	 	
				 if(data==0x01)		
				  {
				   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_SET);
				  }
				else if(data==0x00)		
				  {
					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, GPIO_PIN_RESET);
				  }
						break;	
       case 0x1001:				
					if(data==0x01)		
						{
						 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
						}
					else if(data==0x00)
					 {	
						 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	
					 }	
				break;	
		   case 0x1002:				
					if(data==0x01)		
					{
					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
					}
			   else if(data==0x00)
			   	{	
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);	
			  	}		
				break;	
			case 0x1003:				
					if(data==0x01)		
					{
					 HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);
					}
			   else if(data==0x00)
			   	{	
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);	
			  	}		
				break;	
      default :
						break;					
		   }
      usart1_counter=0;			
	}
 }
	if((USART1->ISR & (1<<3)) == (1<<3))//ORE
	{
		USART1->ICR =1<<3;
	}
}
/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  uint8_t Res;
   if((USART3->ISR & 1<<5) == 1<<5)//接收寄存器数据不为空
       {
					Res=USART3->RDR;
					uart_receive_input(Res);
       }   
}
/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&htim3);
  Display_Flag=1; 

}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
