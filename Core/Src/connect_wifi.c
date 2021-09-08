#include "stm32l4xx.h"
#include "wifi.h"
#include "delay.h"
#include "connect_wifi.h"
  
void Connect_Wifi(void)
{
 if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(WIFI_KEY_GPIO_Port, WIFI_KEY_Pin))
		{
			delay_ms(10);
			if (GPIO_PIN_RESET == HAL_GPIO_ReadPin(WIFI_KEY_GPIO_Port, WIFI_KEY_Pin))
				{
					mcu_set_wifi_mode(0);
					printf("begin connect wifi\r\n");
				}
		}
	switch(mcu_get_wifi_work_state())
	{
		 case SMART_CONFIG_STATE:       
					 printf("smart config\r\n");
				 HAL_GPIO_TogglePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin);
				 delay_ms(250);
			break;
			case AP_STATE:        
					 printf("AP config\r\n");
					 HAL_GPIO_TogglePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin);
					 delay_ms(500);
			break;
			case WIFI_NOT_CONNECTED: 
					 printf("connect wifi\r\n");
					 HAL_GPIO_WritePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin, GPIO_PIN_SET);	 
			break;
			case WIFI_CONNECTED:
					 printf("connect success\r\n");
					 HAL_GPIO_WritePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin, GPIO_PIN_SET);	 
			case WIFI_CONN_CLOUD:
					 HAL_GPIO_WritePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin, GPIO_PIN_SET);
			break;
			default:
					 HAL_GPIO_WritePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin, GPIO_PIN_RESET);	 
					 printf ("connect fail\r\n");
			break;
				
	}
}

