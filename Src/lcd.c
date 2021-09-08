/************************************File Info*********************************************************
** File name:               lcd.c
** Descriptions:            ���Ĵ�����ͨ������
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
** Description    : ����д�봥���������Ĵ���
** Input          : uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length
** Output         : None
** Return         : None
** Attention		 	: 
*******************************************************************************/
void WriteDataToLCD(uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length)
{
  /*����ĳ�����֡ͷ��2���ֽڣ�+���ݳ��ȣ�1���ֽڣ�+ָ�1���ֽڣ�+��ʼ��ַ��2���ֽڣ�+���ݣ�����Ϊlength��*/
	uint8_t i;
	usart1_txBuf[0]=0x5a;
	usart1_txBuf[1]=0xa5;
	usart1_txBuf[2]=length+3;
	usart1_txBuf[3]=0x82;
	usart1_txBuf[4]=(uint8_t)((startAddress>>8)&0xff);//��ʼ��ַ
	usart1_txBuf[5]=(uint8_t)(startAddress&0XFF);//��ʼ��ַ
	for(i=0;i<length;i++)
	{
		usart1_txBuf[i+6]=((SEND_BUF[i+return_data_start_addr]));
	}
	HAL_UART_Transmit(&huart1, usart1_txBuf, length+6, 20);
}
/*******************************************************************************
** Function Name  :void ReadDataFromLCD(uint16_t startAddress,uint8_t readWordLength)
** Description    : �������洢������
** Input          : uint16_t startAddress,uint8_t readWordLength
** Output         : None
** Return         : None
** Attention		 	: 
*******************************************************************************/
void ReadDataFromLCD(uint16_t startAddress,uint16_t readWordLength)
{
  //����ĳ�����֡ͷ��2���ֽڣ�+���ݳ��ȣ�1���ֽڣ�+ָ�1���ֽڣ�+��ʼ��ַ��2���ֽڣ�+��ȡ���ֳ��ȣ�1���ֽڣ�
	usart1_txBuf[0]=0x5a;
	usart1_txBuf[1]=0xa5;
	usart1_txBuf[2]=0x04;
	usart1_txBuf[3]=0x83;
	usart1_txBuf[4]=(uint8_t)((startAddress>>8)&0xff);//��ʼ��ַ
	usart1_txBuf[5]=(uint8_t)(startAddress&0xff);//��ʼ��ַ
	usart1_txBuf[6]=readWordLength;//��ȡ����

	HAL_UART_Transmit(&huart1, usart1_txBuf, 7 , 20);
}

/*******************************************************************************
** Function Name  :void send_tz(void))
** Description    : ��תҳ�溯��
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
