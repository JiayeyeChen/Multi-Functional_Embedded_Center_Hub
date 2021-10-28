#ifndef __PWM_H
#define __PWM_H 
 
 
#include "stm32f4xx_hal.h"
#include "lcd_rgb.h" 
 
#define  LTDC_PWM_PIN								GPIO_PIN_13								// 引脚			
#define	LTDC_PWM_PORT								GPIOD										// 端口
#define 	GPIO_LTDC_PWM_CLK_ENABLE        	   __HAL_RCC_GPIOD_CLK_ENABLE()	 	// 时钟
#define  LCD_PWM_AF           					GPIO_AF2_TIM4							// 复用
	
#define  LTDC_PWM_TIM								TIM4										// 定时器
#define  LTDC_PWM_TIM_CLK_ENABLE					__HAL_RCC_TIM4_CLK_ENABLE()		// 时钟
#define  LTDC_PWM_TIM_CHANNEL					 	TIM_CHANNEL_2							// 通道


/*-------------------- 函数声明 ----------------------*/

void LCD_PWMinit(uint8_t pulse);			 //PWM初始化
void LCD_PwmSetPulse (uint8_t pulse);	 //设置占空比


#endif //__PWM_H



