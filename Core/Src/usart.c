/***
	************************************************************************************************
	*	@file  	usart.c
	*	@version V1.0
	*  @date    2019-12-25
	*	@author  反客科技
	*	@brief   usart相关函数
   ************************************************************************************************
   *  @description
	*
	*	实验平台：反客STM32F429BIT6核心板 （型号：FK429M1）
	*	淘宝地址：https://shop212360197.taobao.com
	*	QQ交流群：536665479
	*
>>>>> 文件说明：
	*
	*  初始化usart引脚，配置波特率等参数
	*
	************************************************************************************************
***/


#include "usart.h"


UART_HandleTypeDef huart1;  // UART_HandleTypeDef 结构体变量


/*************************************************************************************************
*	函 数 名:	HAL_UART_MspInit
*	入口参数:	huart - UART_HandleTypeDef定义的变量，即表示定义的串口
*	返 回 值:无
*	函数功能:	初始化串口引脚
*	说    明:无		
*************************************************************************************************/

/*************************************************************************************************
*	函 数 名:	USART1_Init
*	入口参数:	无
*	返 回 值:无
*	函数功能:	初始化串口配置
*	说    明:无		 
*************************************************************************************************/

void USART1_Init(void)
{
	huart1.Instance 				= USART1;						// USART1
	huart1.Init.BaudRate 		= USART1_BaudRate;			// 波特率
	huart1.Init.WordLength 		= UART_WORDLENGTH_8B;		// 8位数据宽度
	huart1.Init.StopBits			= UART_STOPBITS_1;			// 停止位1
	huart1.Init.Parity 			= UART_PARITY_NONE;			// 无校验
	huart1.Init.Mode 				= UART_MODE_TX;				// 发送模式
	huart1.Init.HwFlowCtl 		= UART_HWCONTROL_NONE;		// 不启用硬件流控制
	huart1.Init.OverSampling	= UART_OVERSAMPLING_16;		// 采样时钟周期16
	
	if (HAL_UART_Init(&huart1) != HAL_OK)	// 初始化配置
	{

	}
  
  GPIO_InitTypeDef GPIO_InitStruct = {0};
	

  __HAL_RCC_USART1_CLK_ENABLE();		// 开启 USART1 时钟

  GPIO_USART1_TX_CLK_ENABLE;				// 开启 USART1 TX 引脚的 GPIO 时钟
  GPIO_USART1_RX_CLK_ENABLE;				// 开启 USART1 RX 引脚的 GPIO 时钟

  GPIO_InitStruct.Pin 			= USART1_TX_PIN;					// TX引脚
  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;				// 复用推挽输出
  GPIO_InitStruct.Pull 		= GPIO_PULLUP;						// 上拉
  GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;	// 速度等级 100M
  GPIO_InitStruct.Alternate 	= GPIO_AF7_USART1;				// 复用为USART1
  HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin 			= USART1_RX_PIN;					// RX引脚
  HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);		
}

/*************************************************************************************************
*	函 数 名:	fputc
*	入口参数:	ch - 要输出的字符 ，  f - 文件指针（这里用不到）
*	返 回 值:正常时返回字符，出错时返回 EOF（-1）
*	函数功能:	重定向 fputc 函数，目的是使用 printf 函数
*	说    明:无		
*************************************************************************************************/

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);	// 发送单字节数据
	return (ch);
}

