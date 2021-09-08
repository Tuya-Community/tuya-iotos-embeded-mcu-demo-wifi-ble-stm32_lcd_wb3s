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
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F103������
//��ʱ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2019/9/17
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
extern TIM_HandleTypeDef TIM3_Handler;      //��ʱ����� 

void TIM3_Init(uint16_t arr,uint16_t psc);
#endif
