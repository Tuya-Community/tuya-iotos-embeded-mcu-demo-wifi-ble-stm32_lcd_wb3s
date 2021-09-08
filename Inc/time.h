/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
//  */
///* Define to prevent recursive inclusion -------------------------------------*/
//#ifndef __TIM_H__
//#define __TIM_H__

//#ifdef __cplusplus
//extern "C" {
//#endif

///* Includes ------------------------------------------------------------------*/
//#include "stm32l4xx_hal.h"

///* USER CODE BEGIN Includes */

///* USER CODE END Includes */

////extern TIM_HandleTypeDef htim6;
//extern TIM_HandleTypeDef htim7;

///* USER CODE BEGIN Private defines */

///* USER CODE END Private defines */

////void MX_TIM6_Init(void);
//void MX_TIM7_Init(void);

///* USER CODE BEGIN Prototypes */

///* USER CODE END Prototypes */

//#ifdef __cplusplus
//}
//#endif

//#endif /* __TIM_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#ifndef _TIMER_H
#define _TIMER_H
#include "stm32l4xx_hal.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F103开发板
//定时器驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2019/9/17
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 

void TIM3_Init(uint16_t arr,uint16_t psc);
#endif
