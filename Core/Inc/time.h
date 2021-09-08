/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
  ******************************************************************************/
#ifndef _TIMER_H
#define _TIMER_H
#include "stm32l4xx_hal.h"	

extern TIM_HandleTypeDef htim3;
void MX_TIM3_Init(uint16_t arr,uint16_t psc);

#endif
