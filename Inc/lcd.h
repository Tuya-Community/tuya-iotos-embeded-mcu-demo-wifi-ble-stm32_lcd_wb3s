#ifndef __LCD_H
#define __LCD_H	
#include "stdint.h"

extern uint8_t SEND_BUF[20]; 
void WriteDataToLCD(uint16_t startAddress,uint16_t return_data_start_addr,uint16_t length);
void ReadDataFromLCD(uint16_t startAddress,uint16_t readWordLength);
void send_tz(void);
#endif













