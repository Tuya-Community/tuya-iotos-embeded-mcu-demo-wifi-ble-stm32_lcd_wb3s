/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "main.h"
#include "sx126x_v01.h"
#include "usart.h"
#include "lcd.h"
#include "delay.h"
#include "wifi.h"
#include "connect_wifi.h"
#include "time.h"

uint8_t SEND_BUF[20] = {0}; 

typedef union
{   
        double  f_type;
        uint8_t val[8];
}FL_TYPE;
FL_TYPE hh;

SPI_HandleTypeDef hspi1;

#define LORA_MODE	1
#define FSK_MODE   0

#define TRANSMITTER    0
#define RECEIVER       1

#define RX_CONTINOUS    1  //连续接收

#if (TRANSMITTER == RECEIVER)
    #error "Please define only Transmitter or receiver."
#endif

#define TEST_MODE	3  	//0-infinite preamble TX mode（暂时只是做了lora）
						          //1-continous CW TX 
						          //其他值才能进入到收发分离模式
						
#if (LORA_MODE == FSK_MODE)
    #error "Please define only LoRa or FSK."
#endif


#define TX_OUTPUT_POWER                             22        // dBm  //测出来是18.536
#define RF_FREQUENCY                                490000000//480000000//915000000//470000000 // Hz


#if (FSK_MODE==1)

#define FSK_FDEV                                    10000//38400//25e3      // Hz 
#define FSK_DATARATE                                19200//40e3      // bps
#define FSK_BANDWIDTH                               93800//93800////58600  140e3     // Hz >> DSB in sx126x
#define FSK_AFC_BANDWIDTH                           100000     // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   true
#define FSK_FIX_LENGTH_PAYLOAD 						          10
#define FSK_CRC										true
 
#elif (LORA_MODE==1)

#define LORA_BANDWIDTH                              1      // [0: 125 kHz,
															//	1: 250 kHz, 														 
															//	2: 500k
															//	3 :20.83kHz
															//	4:31.25kHz
															//	5:62.5kHz4
															//6:41.67
#define LORA_SPREADING_FACTOR                       10        // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx  SF5&6 will automatilly change to 12
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#endif


#define HAL_MAX_DELAY      0xFFFFFFFFU

#define RX_TIMEOUT_VALUE                            1000
#define TX_TIMEOUT                                  65535 
#define BUFFER_SIZE                                 250//10//250 // Define the payload size here

#define CADTIMEOUT_MS								2000   //CAD timeout 时间  用ms表示

uint8_t dat;
uint8_t cnt=0x55;
uint8_t recdat=0;
uint8_t version=0;

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE]={0};

int8_t RssiValue = 0;
int8_t SnrValue = 0;

PacketStatus_t RadioPktStatus;
uint8_t RadioRxPayload[255];
uint8_t RadioRxPacketSize;
uint8_t SendCnt=0;

volatile bool TXDone=false;
volatile bool RXDoneFlag=false;
volatile bool TimeOutFlag=false;
volatile bool CRCFail=false;

volatile int Cnt1=0;

const RadioLoRaBandwidths_t Bandwidths_copy[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500,LORA_BW_020,LORA_BW_031,LORA_BW_062,LORA_BW_041 };

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);

int main(void)
{

  uint8_t i=0;
  bool DetectTruetable[100]={0};//CAD成功的分布
  bool RXTruetable[100]={0};//CAD后能接收正确的分布
  uint8_t CadDetectedTime=0;//检测到的cad的次数
  uint8_t RxCorrectTime=0;  //RX 接收正确次数
  uint8_t TxTime=0;		      //TX 次数

	//连续发送的时候用
	uint8_t ModulationParam[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  uint8_t PacketParam[9] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
	MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
	wifi_protocol_init();   //wifi协议初始化
  TIM3_Init(20-1,8000-1); //定时器3初始化，定时2ms
	
  for(i=0;i<1;i++)
  {
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
	HAL_Delay(1000);
  }
	
  SX126xReset();
  i=SX126xReadRegister(REG_LR_CRCSEEDBASEADDR);
  if(i==0x1D)
  {   
	printf("SPI SUCCESS!\n\r");
  }
  else
  {
	printf("SPI Fail! REG_LR_CRCSEEDBASEADDR=%x\n\r",i);
  }
  RadioInit();
  SX126xWriteRegister(0x889, SX126xReadRegister(0x889) & 0xfB);//SdCfg0 (0x889) sd_res (bit 2) = 0 
  printf("RadioInit Done!\n\r");


#if (TEST_MODE==0)   //infinite preamble TX mode
	//连续发送
	SX126xSetStandby( STDBY_RC );
	SX126xSetPacketType(PACKET_TYPE_LORA);//todo: 增加发射FSK模式下的改指令
	
	printf("set lora params\n");
	ModulationParam[0]=LORA_SPREADING_FACTOR;
	ModulationParam[1]=Bandwidths_copy[LORA_BANDWIDTH];
	ModulationParam[2]=LORA_CODINGRATE;
	ModulationParam[3]=0;//1:SF11 and SF12 0:其他 低速率优化  
	SX126xWriteCommand( RADIO_SET_MODULATIONPARAMS, ModulationParam, 4 );//lora发射参数配置


	//设置lora包参数
	PacketParam[0]=(LORA_PREAMBLE_LENGTH>>8)& 0xFF;
	PacketParam[1]=LORA_PREAMBLE_LENGTH;
	PacketParam[2]=LORA_FIX_LENGTH_PAYLOAD_ON;//head type
	PacketParam[3]=0xFF;//0Xff is MaxPayloadLength
	PacketParam[4]=true;//CRC on
	PacketParam[5]=LORA_IQ_INVERSION_ON;
	SX126xWriteCommand( RADIO_SET_PACKETPARAMS, PacketParam, 6 );

	//SX126xWriteBuffer( 0x00, SendData, 10 );

	//连续发送lora
	SX126xSetRfFrequency( RF_FREQUENCY );
    SX126xSetRfTxPower( TX_OUTPUT_POWER );
	SX126xSetTxInfinitePreamble();

	printf("TxContinuousWave Now--infinite preamble!\n\r");
	while(1);
#elif (TEST_MODE==1) //TX CW

	RadioSetTxContinuousWave( RF_FREQUENCY, TX_OUTPUT_POWER, TX_TIMEOUT );
	printf("TxContinuousWave Now---CW!\n\r");
	while(1);

#endif


#if (FSK_MODE==1)

	SX126xSetRfFrequency(RF_FREQUENCY);
	RadioSetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, FSK_BANDWIDTH,
						FSK_DATARATE, 0,
						FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
						true, 0, 0, 0, 3000 );
	
	RadioSetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
						0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
						0, FSK_FIX_LENGTH_PAYLOAD_ON, FSK_FIX_LENGTH_PAYLOAD, FSK_CRC,0, 0,false, RX_CONTINOUS );
	
	printf("FSK:%d,Fdev=%ld,BitRate=%ld,BW=%ld,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,FSK_FDEV,FSK_DATARATE,FSK_BANDWIDTH,TX_OUTPUT_POWER,FSK_PREAMBLE_LENGTH,BUFFER_SIZE);
	printf("configure FSK parameters done\n!");

	

#elif (LORA_MODE==1)

			SX126xSetRfFrequency(RF_FREQUENCY);
			RadioSetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
									   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
									   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
									   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );


		  RadioSetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
										 LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
										 LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
										 0, true, 0, 0, LORA_IQ_INVERSION_ON, RX_CONTINOUS );//最后一个参数设置是否是连续接收


		
		  printf("LORA:%d,SF=%d,codeRate=%d,BW=%d,PWR=%d,PreLen=%d,PYLOAD=%d\n\r",RF_FREQUENCY,LORA_SPREADING_FACTOR,LORA_CODINGRATE,LORA_BANDWIDTH,TX_OUTPUT_POWER,LORA_PREAMBLE_LENGTH,BUFFER_SIZE);
		  if (RadioPublicNetwork.Previous==true && RadioPublicNetwork.Current==false)
			printf("public\n\r");
		  else if (RadioPublicNetwork.Previous==false && RadioPublicNetwork.Current==false)
			printf("private\n\r");
	    printf("configure LORA parameters done\n!");
		 
#endif

  while (1)
  {
    
#if (TRANSMITTER==1)

	while(1)
	{
		  Buffer[0] = TxTime++;
	    Buffer[1] = 1;
	    Buffer[2] = 2;
	    Buffer[3] = 3;
	    Buffer[4] = 0;
	    Buffer[5] = 0;
		  RadioSend(Buffer,20);
		while(TXDone==false && TimeOutFlag==false);//一直等待tx done
		TXDone=false;
		TimeOutFlag=false;
		printf("TxTime=%d\n",TxTime);
		HAL_Delay(500); ///1s

		//读取状态
		RadioStatus=SX126xGetStatus();
		printf("RadioStatus is(after TX_DONE) %d\n",(((RadioStatus.Value)>>4)&0x07));
		
		
	}
#elif (RECEIVER==1) 
 while(1)
	{	
#if (RX_CONTINOUS==1)
		//开始接收	
	 RadioRx(0xFFFFFF);//50MS(0XC80)超时  0-单次接收 无超时
   printf("continous RX...\n");
   while(1);//连续接收
#endif
 		RadioRx(2000);//50MS(0XC80)超时  0-单次接收 无超时
  	while(RXDoneFlag==false && TimeOutFlag==false && CRCFail==false);
		if(RXDoneFlag==true || TimeOutFlag==true || CRCFail==true)
		{
			if(CRCFail==false)	//CRC无错误
			{
				if(RXDoneFlag==true)
				{
					printf("\n%d:RxCorrect-PING\n",RxCorrectTime);
					RxCorrectTime++;
				}
			}
			CRCFail=false;
			RXDoneFlag=false;
			TimeOutFlag=false;
		}
	}
#endif		  
 }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
  /**Initializes the CPU, AHB and APB busses clocks */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	
	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
	
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
    /**Configure the main internal regulator output voltage*/
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
	
   /**Configure the Systick interrupt time */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);
	
    /**Configure the Systick*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
	
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}


/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
	
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SW_CTL1_Pin|SW_CTL2_Pin|TrigIO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, NRESET_Pin|DeviceSel_Pin|SPI_CS_Pin|ANT_SWITCH_POWER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(WIFI_LED_GPIO_Port, WIFI_LED_Pin, GPIO_PIN_RESET);
	
  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
	
	 /*Configure GPIO pin : WIFI_KEY_Pin */
  GPIO_InitStruct.Pin = WIFI_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(WIFI_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_CTL1_Pin SW_CTL2_Pin TrigIO_Pin */
  GPIO_InitStruct.Pin = SW_CTL1_Pin|SW_CTL2_Pin|TrigIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : NRESET_Pin DeviceSel_Pin SPI_CS_Pin ANT_SWITCH_POWER_Pin */
  GPIO_InitStruct.Pin = NRESET_Pin|DeviceSel_Pin|SPI_CS_Pin|ANT_SWITCH_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : BUSY_Pin */
  GPIO_InitStruct.Pin = BUSY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUSY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO1_Pin */
  GPIO_InitStruct.Pin = DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2（配网指示灯） */
  GPIO_InitStruct.Pin = WIFI_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(WIFI_LED_GPIO_Port, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}
/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	SX126xOnDio1Irq();
}

//DIO1的中断函数
void SX126xOnDio1Irq(void)
{
	 uint16_t irqRegs = SX126xGetIrqStatus( );
     SX126xClearIrqStatus( IRQ_RADIO_ALL );//这里清掉中断标志
     //发送结束
	 if( ( irqRegs & IRQ_TX_DONE ) == IRQ_TX_DONE )
        {
		   TXDone=true;
		   HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		   OnTxDone();			
        }
		//在SX126xSetTx()设置了一个超时时间 可以检测改功能 --ok
 		if( ( irqRegs & IRQ_RX_TX_TIMEOUT ) == IRQ_RX_TX_TIMEOUT )
        {
		   TimeOutFlag=true;
		   printf(" RX/TX timeout\n");
        }
        if( ( irqRegs & IRQ_RX_DONE ) == IRQ_RX_DONE )
        {
            SX126xGetPayload( RadioRxPayload, &RadioRxPacketSize , 255 );
            SX126xGetPacketStatus( &RadioPktStatus );

			      HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
           //OnRxDone();
            RXDoneFlag=true;
      }

      if( ( irqRegs & IRQ_CRC_ERROR ) == IRQ_CRC_ERROR )
        {
                printf("CRC fail\n");
                CRCFail=true;
        }
      if( ( irqRegs & IRQ_CAD_DONE ) == IRQ_CAD_DONE )
      {
			if ( ( irqRegs & IRQ_CAD_ACTIVITY_DETECTED ) == IRQ_CAD_ACTIVITY_DETECTED ) 
			{
           //printf("IRQ_CAD_ACTIVITY_DETECTED\n");	
           //CadDetect=true;
			}            
      }
      if( ( irqRegs & IRQ_PREAMBLE_DETECTED ) == IRQ_PREAMBLE_DETECTED )
        {
            __NOP( );
        }
      if( ( irqRegs & IRQ_SYNCWORD_VALID ) == IRQ_SYNCWORD_VALID )
        {
            __NOP( );
        }
      if( ( irqRegs & IRQ_HEADER_VALID ) == IRQ_HEADER_VALID )
        {
            __NOP( );
        }    
}
//产生触发脉冲
void GenTrig(void)
{
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_SET);
	HAL_GPIO_WritePin(TrigIO_GPIO_Port,TrigIO_Pin,GPIO_PIN_RESET);	
}

void OnTxDone(void)
{
	SleepParams_t params = { 0 };
  params.Fields.WarmStart = 1;//热启动
  //printf("params.value=%d\n",params.Value);
  //SX126xSetSleep( params );//热启动可以保存进入睡眠前的状态的相关寄存器  sleep可以清掉所有中断标志位
	printf("OnTxDone\n");
}
void OnRxDone()
{
    int i;
	  uint16_t 	Res;
	  uint16_t light;
		delay_ms(100);
	  printf("onRXDone\n");
    SleepParams_t params = { 0 };

    params.Fields.WarmStart = 1;//热启动

	//TODO:处理包的数据
	if (RadioPktStatus.packetType==PACKET_TYPE_LORA)
	printf("LoRa:PacketSIZE=%d,RSSI=%d,SNR=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.LoRa.RssiPkt, 
							RadioPktStatus.Params.LoRa.SnrPkt);
	else if (RadioPktStatus.packetType==PACKET_TYPE_GFSK)
	printf("FSK:PacketSIZE=%d,RssiAvg=%d,RssiSync=%d\n",
			RadioRxPacketSize,
					RadioPktStatus.Params.Gfsk.RssiAvg, 
							RadioPktStatus.Params.Gfsk.RssiSync);
	printf("Payload: ");
	for(i=0;i<RadioRxPacketSize;i++)
	{
		printf("0x%x ",RadioRxPayload[i]);
	}
	//收到的数据送到串口屏和APP上显示
  memset(hh.val,0,8);
  for(i=0;i<8;i++)
	 {
		SEND_BUF[i] = RadioRxPayload[7-i];
		hh.val[i]=RadioRxPayload[i];	 
	 }
	WriteDataToLCD(0X1110,0,8);
	Res=hh.f_type*100;
	delay_ms(5);
	mcu_dp_value_update(DPID_TEMPERATURE,Res); 
	 
	memset(hh.val,0,8);  
	for(i=0;i<8;i++)
	{
		SEND_BUF[i] = RadioRxPayload[15-i];
		 hh.val[i]=RadioRxPayload[8+i];	 
	}
	WriteDataToLCD(0X1118,0,8);
	delay_ms(5);
  Res=hh.f_type*100;
	mcu_dp_value_update(DPID_HUMIDITY,Res);
	memset(hh.val,0,8); 
	
	for(i=0;i<2;i++)
	{
		SEND_BUF[i] = RadioRxPayload[17-i];
	}
  WriteDataToLCD(0X1120,0,2);
	delay_ms(5);
	light=RadioRxPayload[17]*256+RadioRxPayload[16];
	mcu_dp_value_update(DPID_ILLUMINANCE,light);
}
	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
