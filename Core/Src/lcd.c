/************************************File Info*********************************************************
** File name:               lcd.c
** Descriptions:            迪文串口屏通信驱动
**
**--------------------------------------------------------------------------------------------------------
** Created by:              LDL
** Created date:            20210806
** Version:                 v1.0
** Descriptions:            The original version
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "lcd.h"
#include "usart.h"
extern uint8_t usart1_rxBuf[50];
uint8_t usart1_txBuf[50];
/*******************************************************************************
** Function Name  :void WriteDataToLCD(uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length)
** Description    : 数据写入触摸屏变量寄存器
** Input          : uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length
** Output         : None
** Return         : None
** Attention		 	: 
*******************************************************************************/
void WriteDataToLCD(uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length)
{
  /*命令的长度由帧头（2个字节）+数据长度（1个字节）+指令（1个字节）+起始地址（2个字节）+数据（长度为length）*/
	uint8_t i;
	usart1_txBuf[0]=0x5a;
	usart1_txBuf[1]=0xa5;
	usart1_txBuf[2]=length+3;
	usart1_txBuf[3]=0x82;
	usart1_txBuf[4]=(uint8_t)((startAddress>>8)&0xff);//起始地址
	usart1_txBuf[5]=(uint8_t)(startAddress&0XFF);//起始地址
	for(i=0;i<length;i++)
	{
		usart1_txBuf[i+6]=((SEND_BUF[i+return_data_start_addr]));
	}
	HAL_UART_Transmit(&huart1, usart1_txBuf, length+6, 20);
}
/*******************************************************************************
** Function Name  :void ReadDataFromLCD(uint16_t startAddress,uint8_t readWordLength)
** Description    : 读变量存储器数据
** Input          : uint16_t startAddress,uint8_t readWordLength
** Output         : None
** Return         : None
** Attention		 	: 
*******************************************************************************/
void ReadDataFromLCD(uint16_t startAddress,uint16_t readWordLength)
{
  //命令的长度由帧头（2个字节）+数据长度（1个字节）+指令（1个字节）+起始地址（2个字节）+读取的字长度（1个字节）
	usart1_txBuf[0]=0x5a;
	usart1_txBuf[1]=0xa5;
	usart1_txBuf[2]=0x04;
	usart1_txBuf[3]=0x83;
	usart1_txBuf[4]=(uint8_t)((startAddress>>8)&0xff);//起始地址
	usart1_txBuf[5]=(uint8_t)(startAddress&0xff);//起始地址
	usart1_txBuf[6]=readWordLength;//读取长度

	HAL_UART_Transmit(&huart1, usart1_txBuf, 7 , 20);
}

/*******************************************************************************
** Function Name  :void send_tz(void))
** Description    : 跳转页面函数
** Input          : None
** Output         : None
** Return         : None
** Attention		 	: 
*******************************************************************************/
void send_tz(void)
{
	uint8_t i;
	usart1_txBuf[0]=0x5a;
	usart1_txBuf[1]=0xa5;
	usart1_txBuf[2]=0x07;
	usart1_txBuf[3]=0x82;
	usart1_txBuf[4]=0x00;
	usart1_txBuf[5]=0x84;
	usart1_txBuf[6]=0x5a;
	usart1_txBuf[7]=0x01;
	for(i=0;i<2;i++)
	{
		usart1_txBuf[i+8]=((SEND_BUF[i]));
	}
	HAL_UART_Transmit(&huart1, usart1_txBuf, 10, 20);
}
