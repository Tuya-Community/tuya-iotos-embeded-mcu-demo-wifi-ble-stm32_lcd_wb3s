/**
  ******************************************************************************
  * @file    stm32l4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"
#include "stm32l4xx.h"
#include "stm32l4xx_it.h"
#include "wifi.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

uint8_t usart1_rxBuf[50];

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32L4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l4xx.s).                    */
/******************************************************************************/
/**
  * @brief  This function  handles USART1 global interrupt.
  * @retval None
  */


void USART1_IRQHandler(void)
{
	uint8_t i;
	uint8_t usart1_data;
	uint8_t send[2]={0x5a,0xa5};
	uint16_t addr=0,data=0,len=0;		//定义地址 长度 及数据
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
			len =  usart1_rxBuf[6];														//获取接收的数据字个数
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
* @brief This function handles EXTI line4 interrupt.
*/

void EXTI4_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_IRQn 0 */
  /* USER CODE END EXTI4_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
  /* USER CODE END EXTI15_10_IRQn 0 */
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
