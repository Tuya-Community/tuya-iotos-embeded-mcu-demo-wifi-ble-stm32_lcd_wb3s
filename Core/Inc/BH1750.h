#ifndef __BH1750_H__
#define __BH1750_H__
#include <stdint.h>

#define	BH1750_ADDR_WRITE	0x46	//01000110
#define	BH1750_ADDR_READ	0x47	//01000111

typedef enum
{
	POWER_OFF_CMD	=	0x00,	 //�ϵ磺�޼���״̬
	POWER_ON_CMD	=	0x01,	 //ͨ�磺�ȴ�����ָ��
	RESET_REGISTER	=	0x07,//�������ּĴ������ڶϵ�״̬�²������ã�
	CONT_H_MODE		=	0x10,	 //����H�ֱ���ģʽ����11x�ֱ����¿�ʼ����������ʱ��120ms
	CONT_H_MODE2	=	0x11,	 //����H�ֱ���ģʽ2����0.51x�ֱ����¿�ʼ����������ʱ��120ms
	CONT_L_MODE		=	0x13,	 //����L�ֱ���ģʽ����411�ֱ����¿�ʼ����������ʱ��16ms
	ONCE_H_MODE		=	0x20,	 //һ�θ߷ֱ���ģʽ����11x�ֱ����¿�ʼ����������ʱ��120ms���������Զ�����Ϊ�ϵ�ģʽ
	ONCE_H_MODE2	=	0x21,	 //һ�θ߷ֱ���ģʽ2����0.51x�ֱ����¿�ʼ����������ʱ��120ms���������Զ�����Ϊ�ϵ�ģʽ
	ONCE_L_MODE		=	0x23	 //һ�εͷֱ���ģʽ����411x�ֱ����¿�ʼ����������ʱ��16ms���������Զ�����Ϊ�ϵ�ģʽ
} BH1750_MODE;

uint8_t	BH1750_Send_Cmd(BH1750_MODE cmd);
uint8_t BH1750_Read_Dat(uint8_t* dat);
uint16_t BH1750_Dat_To_Lx(uint8_t* dat);

#endif

