/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *****************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"
#include "wifi.h"
#include "sht3x.h"
#include "BH1750.h"
#include "connect_wifi.h"
#include "lcd.h"
#include "delay.h"
#include "time.h"

uint8_t SEND_BUF[20] = {0}; 
extern unsigned char Display_Flag; 
void SystemClock_Config(void);


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
 
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick.*/
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	wifi_protocol_init();       //wifi协议初始化
  MX_I2C2_Init();
  MX_I2C1_Init();
	
	MX_TIM3_Init(10000-1,8000-1); //定时器3初始化，定时1s
	
  SHT3x_Reset();
	if( 0 == SHT3x_Init())
		printf("SHT3x_Init OK \r\n");
	else
		printf("SHT3x_Init ERR \r\n");
	

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		  Connect_Wifi();	
			wifi_uart_service();//wifi串口数据处理服务  
		
   if(1 == Display_Flag)
		{
		  Deal_Data_Display();	
			Display_Flag=0;
	  }			
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_I2C2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
void Deal_Data_Display(void)
{
  uint8_t i;
	uint16_t  light;
  double Tem_val,Hum_val;
  uint8_t dat[2] = {0};	
  uint8_t Buffer[30]={0};	
  

	  HAL_Delay(1000);
  	if(SHT3x_Get_Humiture_periodic(&Tem_val,&Hum_val) == 0)
			{
				memcpy(Buffer,(double*)(&Tem_val),8);		
				memcpy(Buffer+8,(double*)(&Hum_val),8);
       for(i=0;i<8;i++)
	      {
		      SEND_BUF[i] = Buffer[7-i];	 
	      }
		    WriteDataToLCD(0X1110,0,8);
		   for(i=0;i<8;i++)
	      {
		      SEND_BUF[i] = Buffer[15-i];	 
	      }
		    WriteDataToLCD(0X1118,0,8);
				
				mcu_dp_value_update(DPID_TEMPERATURE,Tem_val*100); //温度数据上报;
        mcu_dp_value_update(DPID_HUMIDITY,Hum_val*100);    //湿度数据上报;
				printf("temperature=%6.2lf,humidity=%6.2lf\r\n",Tem_val,Hum_val);		
			}
		else
			printf("Get_Humiture ERR\r\n");
		
			  HAL_Delay(180);
        if(HAL_OK == BH1750_Read_Dat(dat))
        {
					  light=BH1750_Dat_To_Lx(dat);
					  memcpy(Buffer+16, (uint16_t*)(&light), sizeof((uint16_t*)(&light)));
					 for(i=0;i<2;i++)
	          {
		         SEND_BUF[i] = Buffer[17-i];	 
	          }
					  WriteDataToLCD(0X1120,0,2);
	          delay_ms(5);
            printf("illuminance: %5d lx\r\n", light);
					  mcu_dp_value_update(DPID_ILLUMINANCE,light*100); //光照度数据上报;        
        }
        else
        {
            printf("recv fail");
        }  
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
