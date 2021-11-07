#ifndef SYSTEM_PERIPHRALS_H
#define SYSTEM_PERIPHRALS_H

#include "main.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "gpio_digital_filtered_input.h"
#include "led_control.h"

#define SPI_SCK_Pin GPIO_PIN_2
#define SPI_SCK_GPIO_Port GPIOE
#define AD7606_RST_Pin GPIO_PIN_3
#define AD7606_RST_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_4
#define SPI_CS_GPIO_Port GPIOE
#define SPI_MISO_Pin GPIO_PIN_5
#define SPI_MISO_GPIO_Port GPIOE
#define SPI_MOSI_Pin GPIO_PIN_6
#define SPI_MOSI_GPIO_Port GPIOE
#define ONBOARD_BUTTON_KEY_Pin GPIO_PIN_8
#define ONBOARD_BUTTON_KEY_GPIO_Port GPIOI
#define AD7606_CS_Pin GPIO_PIN_6
#define AD7606_CS_GPIO_Port GPIOF
#define AD7606_SCK_Pin GPIO_PIN_7
#define AD7606_SCK_GPIO_Port GPIOF
#define AD7606_MISO_Pin GPIO_PIN_8
#define AD7606_MISO_GPIO_Port GPIOF
#define AD7606_MOSI_Pin GPIO_PIN_9
#define AD7606_MOSI_GPIO_Port GPIOF
#define AD7606_BUSY_Pin GPIO_PIN_10
#define AD7606_BUSY_GPIO_Port GPIOF
#define AD7606_BUSY_EXTI_IRQn EXTI15_10_IRQn
#define AD7606_CV_Pin GPIO_PIN_1
#define AD7606_CV_GPIO_Port GPIOC
#define RS422_TX_Pin GPIO_PIN_0
#define RS422_TX_GPIO_Port GPIOA
#define RS422_RX_Pin GPIO_PIN_1
#define RS422_RX_GPIO_Port GPIOA
#define DAC1_OUTPUT_Pin GPIO_PIN_4
#define DAC1_OUTPUT_GPIO_Port GPIOA
#define DAC2_OUTPUT_Pin GPIO_PIN_5
#define DAC2_OUTPUT_GPIO_Port GPIOA
#define RS485_TX_Pin GPIO_PIN_8
#define RS485_TX_GPIO_Port GPIOD
#define RS485_RX_Pin GPIO_PIN_9
#define RS485_RX_GPIO_Port GPIOD
#define ONBOARD_LED_BLUE_Pin GPIO_PIN_12
#define ONBOARD_LED_BLUE_GPIO_Port GPIOD
#define SPI_FLASH_CS_Pin GPIO_PIN_3
#define SPI_FLASH_CS_GPIO_Port GPIOG
#define ONBOARD_LED_YELLOWGREEN_Pin GPIO_PIN_7
#define ONBOARD_LED_YELLOWGREEN_GPIO_Port GPIOG
#define UART_RX_Pin GPIO_PIN_7
#define UART_RX_GPIO_Port GPIOC
#define ONBOARD_UART_TX_Pin GPIO_PIN_9
#define ONBOARD_UART_TX_GPIO_Port GPIOA
#define ONBOARD_UART_RX_Pin GPIO_PIN_10
#define ONBOARD_UART_RX_GPIO_Port GPIOA
#define RS232_TX_Pin GPIO_PIN_5
#define RS232_TX_GPIO_Port GPIOD
#define RS232_RX_Pin GPIO_PIN_6
#define RS232_RX_GPIO_Port GPIOD
#define UART_TX_Pin GPIO_PIN_14
#define UART_TX_GPIO_Port GPIOG
#define SPI_FLASH_SCK_Pin GPIO_PIN_3
#define SPI_FLASH_SCK_GPIO_Port GPIOB
#define SPI_FLASH_MISO_Pin GPIO_PIN_4
#define SPI_FLASH_MISO_GPIO_Port GPIOB
#define SPI_FLASH_MOSI_Pin GPIO_PIN_5
#define SPI_FLASH_MOSI_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

void SystemPeriphral_Init(void);
void MX_USB_DEVICE_Init(void);
void Button_Init(void);
void LED_Init(void);

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CRC_HandleTypeDef hcrc;
extern DAC_HandleTypeDef hdac;
extern I2C_HandleTypeDef hi2c1;
extern SD_HandleTypeDef hsd;
extern USBD_HandleTypeDef hUsbDeviceHS;

extern GPIOStruct hButtonOnboardKey;
extern LEDHandle  hLEDBlue, hLEDYellowGreen;
#endif
