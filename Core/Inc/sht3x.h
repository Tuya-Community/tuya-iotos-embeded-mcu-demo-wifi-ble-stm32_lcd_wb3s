#ifndef __SHT3X_H_
#define __SHT3X_H_

#include "stdint.h"

uint8_t SHT3x_Init(void);	/* 描述：SHT3x初始化函数  */
uint8_t SHT3x_ReadSerialNumber(uint32_t* serialNumber);
uint8_t SHT3x_Get_Humiture_single(double *Tem_val,double *Hum_val);	/* 单次获取 */
uint8_t SHT3x_Get_Humiture_periodic(double *Tem_val,double *Hum_val); /* 周期获取 */
void SHT3x_Reset(void);

/* 枚举SHT3x命令列表 */
typedef enum
{
    /* 软件复位命令 */
    SOFT_RESET_CMD = 0x30A2,	
    /* 单次测量模式
    命名格式：Repeatability_CS_CMD
    CS： Clock stretching */
    HIGH_ENABLED_CMD    = 0x2C06,
    MEDIUM_ENABLED_CMD  = 0x2C0D,
    LOW_ENABLED_CMD     = 0x2C10,
    HIGH_DISABLED_CMD   = 0x2400,
    MEDIUM_DISABLED_CMD = 0x240B,
    LOW_DISABLED_CMD    = 0x2416,

    /* 周期测量模式
    命名格式：Repeatability_MPS_CMD
    MPS：measurement per second */
    HIGH_0_5_CMD   = 0x2032,
    MEDIUM_0_5_CMD = 0x2024,
    LOW_0_5_CMD    = 0x202F,
    HIGH_1_CMD     = 0x2130,
    MEDIUM_1_CMD   = 0x2126,
    LOW_1_CMD      = 0x212D,
    HIGH_2_CMD     = 0x2236,
    MEDIUM_2_CMD   = 0x2220,
    LOW_2_CMD      = 0x222B,
    HIGH_4_CMD     = 0x2334,
    MEDIUM_4_CMD   = 0x2322,
    LOW_4_CMD      = 0x2329,
    HIGH_10_CMD    = 0x2737,
    MEDIUM_10_CMD  = 0x2721,
    LOW_10_CMD     = 0x272A,
	/* 周期测量模式读取数据命令 */
	READOUT_FOR_PERIODIC_MODE = 0xE000,
	/* 读取传感器编号命令 */
	READ_SERIAL_NUMBER = 0x3780,
} SHT3X_CMD;


#endif
