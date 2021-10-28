/***
	************************************************************************************************
	*	@file  	usart.c
	*	@version V1.0
	*  @date    2019-12-25
	*	@author  ���ͿƼ�
	*	@brief   usart��غ���
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F429BIT6���İ� ���ͺţ�FK429M1��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> �ļ�˵����
	*
	*  ��ʼ��usart���ţ����ò����ʵȲ���
	*
	************************************************************************************************
***/


#include "usart.h"


UART_HandleTypeDef huart1;  // UART_HandleTypeDef �ṹ�����


/*************************************************************************************************
*	�� �� ��:	HAL_UART_MspInit
*	��ڲ���:	huart - UART_HandleTypeDef����ı���������ʾ����Ĵ���
*	�� �� ֵ:��
*	��������:	��ʼ����������
*	˵    ��:��		
*************************************************************************************************/

/*************************************************************************************************
*	�� �� ��:	USART1_Init
*	��ڲ���:	��
*	�� �� ֵ:��
*	��������:	��ʼ����������
*	˵    ��:��		 
*************************************************************************************************/

void USART1_Init(void)
{
	huart1.Instance 				= USART1;						// USART1
	huart1.Init.BaudRate 		= USART1_BaudRate;			// ������
	huart1.Init.WordLength 		= UART_WORDLENGTH_8B;		// 8λ���ݿ��
	huart1.Init.StopBits			= UART_STOPBITS_1;			// ֹͣλ1
	huart1.Init.Parity 			= UART_PARITY_NONE;			// ��У��
	huart1.Init.Mode 				= UART_MODE_TX;				// ����ģʽ
	huart1.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;		// ������Ӳ��������
	huart1.Init.OverSampling	= UART_OVERSAMPLING_16;		// ����ʱ������16
	
	if (HAL_UART_Init(&huart1) != HAL_OK)	// ��ʼ������
	{

	}
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	

  __HAL_RCC_USART1_CLK_ENABLE();		// ���� USART1 ʱ��

  GPIO_USART1_TX_CLK_ENABLE;				// ���� USART1 TX ���ŵ� GPIO ʱ��
  GPIO_USART1_RX_CLK_ENABLE;				// ���� USART1 RX ���ŵ� GPIO ʱ��

  GPIO_InitStruct.Pin 			= USART1_TX_PIN;					// TX����
  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;				// �����������
  GPIO_InitStruct.Pull 		= GPIO_PULLUP;						// ����
  GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;	// �ٶȵȼ� 100M
  GPIO_InitStruct.Alternate 	= GPIO_AF7_USART1;				// ����ΪUSART1
  HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin 			= USART1_RX_PIN;					// RX����
  HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);		
}

/*************************************************************************************************
*	�� �� ��:	fputc
*	��ڲ���:	ch - Ҫ������ַ� ��  f - �ļ�ָ�루�����ò�����
*	�� �� ֵ:����ʱ�����ַ�������ʱ���� EOF��-1��
*	��������:	�ض��� fputc ������Ŀ����ʹ�� printf ����
*	˵    ��:��		
*************************************************************************************************/

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);	// ���͵��ֽ�����
	return (ch);
}

