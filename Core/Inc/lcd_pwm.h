#ifndef __PWM_H
#define __PWM_H 
 
 
#include "stm32f4xx_hal.h"
#include "lcd_rgb.h" 
 
#define  LTDC_PWM_PIN								GPIO_PIN_13								// ����			
#define	LTDC_PWM_PORT								GPIOD										// �˿�
#define 	GPIO_LTDC_PWM_CLK_ENABLE        	   __HAL_RCC_GPIOD_CLK_ENABLE()	 	// ʱ��
#define  LCD_PWM_AF           					GPIO_AF2_TIM4							// ����
	
#define  LTDC_PWM_TIM								TIM4										// ��ʱ��
#define  LTDC_PWM_TIM_CLK_ENABLE					__HAL_RCC_TIM4_CLK_ENABLE()		// ʱ��
#define  LTDC_PWM_TIM_CHANNEL					 	TIM_CHANNEL_2							// ͨ��


/*-------------------- �������� ----------------------*/

void LCD_PWMinit(uint8_t pulse);			 //PWM��ʼ��
void LCD_PwmSetPulse (uint8_t pulse);	 //����ռ�ձ�


#endif //__PWM_H



