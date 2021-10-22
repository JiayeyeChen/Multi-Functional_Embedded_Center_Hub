/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ONBOARD_BUTTON_KEY_Pin GPIO_PIN_8
#define ONBOARD_BUTTON_KEY_GPIO_Port GPIOI
#define ONBOARD_LED_BLUE_Pin GPIO_PIN_12
#define ONBOARD_LED_BLUE_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_13
#define LCD_BL_GPIO_Port GPIOD
#define ONBOARD_LED_YELLOWGREEN_Pin GPIO_PIN_7
#define ONBOARD_LED_YELLOWGREEN_GPIO_Port GPIOG
#define ONBOARD_UART_TX_Pin GPIO_PIN_9
#define ONBOARD_UART_TX_GPIO_Port GPIOA
#define ONBOARD_UART_RX_Pin GPIO_PIN_10
#define ONBOARD_UART_RX_GPIO_Port GPIOA
#define SPI_FLASH_SCK_Pin GPIO_PIN_3
#define SPI_FLASH_SCK_GPIO_Port GPIOB
#define SPI_FLASH_MISO_Pin GPIO_PIN_4
#define SPI_FLASH_MISO_GPIO_Port GPIOB
#define SPI_FLASH_MOSI_Pin GPIO_PIN_5
#define SPI_FLASH_MOSI_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
