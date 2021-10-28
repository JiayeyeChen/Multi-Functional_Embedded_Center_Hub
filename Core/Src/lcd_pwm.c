/***
	************************************************************************************************
	*	@file  	lcd_pwm.c
	*	@version V1.0
	*  @date    2020-3-12
	*	@author  ���ͿƼ�
	*	@brief   LCD����pwm��غ���
   ************************************************************************************************
   *  @description
	*
	*	ʵ��ƽ̨������STM32F429BIT6���İ� ���ͺţ�FK429M1��
	*	�Ա���ַ��https://shop212360197.taobao.com
	*	QQ����Ⱥ��536665479
	*
>>>>> �ļ�˵����
	*
	*  1.PWMƵ��Ϊ2KHz
	*	2.HAL_TIM_MspPostInit���ڳ�ʼ��IO�ڣ�HAL_TIM_Base_MspInit���ڿ���ʱ��
	*
	************************************************************************************************
***/


#include "lcd_pwm.h"

TIM_HandleTypeDef htim4;	// TIM_HandleTypeDef �ṹ�����

static uint16_t LCD_PwmPeriod = 500;  		//��ʱ������ֵ

/*************************************************************************************************
*	�� �� ��:	HAL_TIM_MspPostInit
*	��ڲ���:	htim - TIM_HandleTypeDef�ṹ�����������ʾ�����TIM�������
*	�� �� ֵ:��
*	��������:	��ʼ�� TIM ��Ӧ��PWM��
*	˵    ��:�ú����Ĵ����� CubeMX ���汾5.3.0�����ɣ����ڳ�ʼ��PWM�õ������ţ���Ҫע����ǣ��ú���
*				���� MspInit ֮��ĺ��������ᱻ HAL_TIM_Base_Init �������ã���Ҫ�û��Լ�ȥ���á�
*				�û����԰�����ĳ�ʼ���������ϵ� HAL_TIM_Base_MspInit ������Ϊ�˺�CubeMX���ɵĴ���ṹһ�£�
*				�����û��ԱȲο���û�����ϵ�һ��
*
*************************************************************************************************/

/*************************************************************************************************
*	�� �� ��:	LCD_PwmSetPulse
*	��ڲ���:	pulse - PWMռ�ձȣ���Χ 0~100
*	�� �� ֵ:��
*	��������:	����PWMռ�ձ�
*	˵    ��:��
*************************************************************************************************/
	
void  LCD_PwmSetPulse (uint8_t pulse)
{
	uint16_t compareValue ; 
	
	compareValue = pulse * LCD_PwmPeriod / 100; //����ռ�ձ����ñȽ�ֵ

	TIM4->CCR2 = compareValue; 			// �޸�TIM4��ͨ��2�Ƚ�ֵ
}

/*************************************************************************************************
*	�� �� ��:	LCD_PWMinit
*	��ڲ���:	pulse - PWMռ�ձȣ���Χ 0~100
*	�� �� ֵ:��
*	��������:	��ʼ��TIM4,����PWMƵ��Ϊ2KHz
*	˵    ��:�ú����Ĵ����� CubeMX ���汾5.3.0������
*************************************************************************************************/

void  LCD_PWMinit(uint8_t pulse)
{

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	htim4.Instance 					= LTDC_PWM_TIM;							// ��ʱ��
	htim4.Init.Prescaler 			= 89;                              	// Ԥ��Ƶϵ������ʱ��ʱ���ļ���Ƶ��Ϊ 1MKHz
	htim4.Init.CounterMode 			= TIM_COUNTERMODE_UP;               // ���ϼ���ģʽ
	htim4.Init.Period 				= LCD_PwmPeriod -1 ;                // ����ֵ499��������500��
	htim4.Init.ClockDivision 		= TIM_CLOCKDIVISION_DIV1;           // ʱ�Ӳ���Ƶ
	htim4.Init.AutoReloadPreload 	= TIM_AUTORELOAD_PRELOAD_DISABLE;   // ���ƼĴ��� TIMx_CR1 ��ARPE λ��0������ֹ�Զ����ؼĴ�������Ԥװ��

	HAL_TIM_Base_Init(&htim4) ;	// ��������Ĳ�������TIM2���г�ʼ��
  LTDC_PWM_TIM_CLK_ENABLE;		// ���� TIM ʱ��

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;			// ѡ���ڲ�ʱ��Դ
	HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig);           // ��ʼ������ʱ��Դ

	HAL_TIM_PWM_Init(&htim4) ;		// ��������Ĳ�������TIM����PWM��ʼ��   

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;					// �������ѡ�񣬴�ʱ���ø�λģʽ�����Ĵ��� TIMx_CR2 �� MMS ΪΪ000
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;      // ��ʹ�ô�ģʽ
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);    // ��ʼ������

	sConfigOC.OCMode		= TIM_OCMODE_PWM1;											// PWMģʽ1
	sConfigOC.Pulse 		= pulse*LCD_PwmPeriod/100;									// �Ƚ�ֵ250������Ϊ500����ռ�ձ�Ϊ50%
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;										// ��Ч״̬Ϊ�ߵ�ƽ
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;										// ��ֹ����ģʽ
	HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, LTDC_PWM_TIM_CHANNEL);		// ��ʼ������PWM

  GPIO_InitTypeDef GPIO_InitStruct = {0};         // ��ʼ��IO��
  GPIO_LTDC_Black_CLK_ENABLE;			// �����������Ŷ˿�ʱ��

  GPIO_InitStruct.Pin 			= LTDC_PWM_PIN;			// PD13
  GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;		// �����������
  GPIO_InitStruct.Pull 		= GPIO_NOPULL;				// ��������
  GPIO_InitStruct.Speed		= GPIO_SPEED_FREQ_LOW;	// ����ģʽ
  GPIO_InitStruct.Alternate 	= LCD_PWM_AF;				// ����
  
  HAL_GPIO_Init(LTDC_PWM_PORT, &GPIO_InitStruct);		// ��ʼ��IO��
	HAL_TIM_PWM_Start(&htim4,LTDC_PWM_TIM_CHANNEL);		// ����PWM			
}

/*************************************************************************************************
*	�� �� ��:	HAL_TIM_Base_MspInit
*	��ڲ���:	htim_base - TIM_HandleTypeDef�ṹ�����������ʾ�����TIM�������
*	�� �� ֵ:��
*	��������:	����ʱ��
*	˵    ��:�ú����Ĵ����� CubeMX ���汾5.3.0�����ɣ��ᱻ HAL_TIM_Base_Init ���ã��û����԰� TIM
*				��IO�ڳ�ʼ���ŵ��������Ϊ�˺�CubeMX���ɵĴ���ṹһ�£������û��ԱȲο���û�����ϵ�һ��
*
*************************************************************************************************/

//void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
//{
//  if(htim_base->Instance==LTDC_PWM_TIM)
//  {
//    LTDC_PWM_TIM_CLK_ENABLE;		// ���� TIM ʱ��
//  }

//}


