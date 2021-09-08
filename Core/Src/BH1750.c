/**************************************************************************
 * �ļ���  ��BH1750.c
 * ����    �����նȴ���ģ��     
****************************************************************************/
#include "BH1750.h"
#include "i2c.h"
#include "usart.h"
/**
 * @brief	��BH1750����һ��ָ��
 * @param	cmd ���� BH1750����ģʽָ���BH1750_MODE��ö�ٶ��壩
 * @retval	�ɹ�����HAL_OK
*/
uint8_t	BH1750_Send_Cmd(BH1750_MODE cmd)
{
  return HAL_I2C_Master_Transmit(&hi2c1, BH1750_ADDR_WRITE, (uint8_t*)&cmd, 1, 0xFFFF);
}

/**
 * @brief	��BH1750����һ�ι�ǿ����
 * @param	dat ���� �洢����ǿ�ȵĵ�ַ�������ֽ����飩
 * @retval	�ɹ� ���� ����HAL_OK
*/
uint8_t BH1750_Read_Dat(uint8_t* dat)
{
	if(HAL_OK == BH1750_Send_Cmd(ONCE_H_MODE))
				{
					 printf("ok\r\n");
				}
	else
				{
					 printf("fail\r\n");
				}
	return HAL_I2C_Master_Receive(&hi2c1, BH1750_ADDR_READ, dat, 2, 0xFFFF);
}
/**
 * @brief	��BH1750�������ֽ�����ת��Ϊ����ǿ��ֵ��0-65535��
 * @param	dat  ���� �洢����ǿ�ȵĵ�ַ�������ֽ����飩
 * @retval	�ɹ� ���� ���ع���ǿ��ֵ
*/
uint16_t BH1750_Dat_To_Lx(uint8_t* dat)
{
	uint16_t lx = 0;
	lx = dat[0];    //dat[0]�Ǹ��ֽڣ�dat[1]�ǵ��ֽ�
	lx <<= 8;
	lx += dat[1];
	lx = (int)(lx / 1.2);	
	return lx;
}
