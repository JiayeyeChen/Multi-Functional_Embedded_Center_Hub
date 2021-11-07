#ifndef __TOUCH_H
#define __TOUCH_H

#include "stm32f4xx.h"
#include "touch_iic.h"  

/*------------------------------------ ��ض��� -----------------------------------*/  	

#define TOUCH_MAX   5	//���������

typedef struct 
{
	uint8_t  flag;			//	������־λ��Ϊ1ʱ��ʾ�д�������
	uint8_t  num;				//	��������
	uint16_t x[TOUCH_MAX];	//	x����
	uint16_t y[TOUCH_MAX];	//	y����
}TouchStructure;

extern TouchStructure touchInfo;	// �������ݽṹ������	

/*------------------------------------ �Ĵ涨�� -----------------------------------*/  		

#define GT9XX_IIC_RADDR 0xBB			// IIC��ʼ����ַ
#define GT9XX_IIC_WADDR 0xBA

#define GT9XX_READ_ADDR 0x814E		// ������Ϣ�Ĵ���������оƬ�ĵ�ַһ��
#define GT9XX_ID_ADDR 	0x8140		// �������ID�Ĵ���������оƬ�ĵ�ַһ��


#define GT9XX_CFG_ADDR 	0x8047		// �̼�������Ϣ�Ĵ�����������ʼ��ַ
#define GT9XX_Chksum_ADDR  0X80FF	// GT911 У��ͼĴ���

#define GT5688_CFG_ADDR 	0x8050		// GT5688 �̼���Ϣ�Ĵ��������üĴ�����ʼ��ַ
#define GT5688_Chksum_ADDR 0x813C		// GT5688 У��ͼĴ���

/*------------------------------------ �������� -----------------------------------*/  		

uint8_t 	Touch_Init(void);		// ��������ʼ��
void 		Touch_Scan(void);		// ����ɨ��
void  	GT9XX_Reset(void);	// ִ�и�λ����
void 		GT9XX_SendCfg(void);	// ����GT911���ò���
void 		GT9XX_ReadCfg(void);	// ��ȡGT911���ò���

#endif

