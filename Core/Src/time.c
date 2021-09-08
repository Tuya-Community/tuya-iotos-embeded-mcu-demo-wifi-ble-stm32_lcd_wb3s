//// ******************************************************************************
//// * @file    time.c
//// * @brief   This file provides code for the configuration
//// *          of the TIM instances.
//// ******************************************************************************
#include "time.h"
#include "myOS.h"
#include "usart.h"

TIM_HandleTypeDef htim3;  //��ʱ����� 
//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
/* TIM3 init function */
void MX_TIM3_Init(uint16_t arr,uint16_t psc)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim3.Instance = TIM3;                            //ͨ�ö�ʱ��3
  htim3.Init.Prescaler = psc;                       //��Ƶϵ��
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;      //���ϼ�����
  htim3.Init.Period = arr;                          //�Զ�װ��ֵ
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;//ʱ�ӷ�Ƶ����
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
	 HAL_TIM_Base_Start_IT(&htim3); //ʹ�ܶ�ʱ��3�Ͷ�ʱ��3�����жϣ�TIM_IT_UPDATE 

}
//��ʱ���ײ�����������ʱ�ӣ������ж����ȼ�
//�˺����ᱻHAL_TIM_Base_Init()��������
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {
    /* TIM3 clock enable */
    __HAL_RCC_TIM3_CLK_ENABLE();            //ʹ��TIM3ʱ��

    /* TIM3 interrupt Init */
    HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0); //�����ж����ȼ�����ռ���ȼ�0�������ȼ�0
    HAL_NVIC_EnableIRQ(TIM3_IRQn);         //����ITM3�ж�   

  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM3)
  {

    __HAL_RCC_TIM3_CLK_DISABLE();

    /* TIM3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM3_IRQn);

  }
}
