#ifndef MRBA3_ADC_H
#define MRBA3_ADC_H


#include "stm32f4xx_hal.h"

#define AD7606_BUSY_EXTI_IRQn 		    EXTI15_10_IRQn

#define AD7606_CS_PIN                 GPIO_PIN_6
#define AD7606_CS_GPIO_PORT           GPIOF
#define AD7606_CLK_PIN                GPIO_PIN_7
#define AD7606_CLK_GPIO_PORT          GPIOF
#define AD7606_MISO_PIN               GPIO_PIN_8
#define AD7606_MISO_GPIO_PORT         GPIOF
#define AD7606_BUSY_PIN               GPIO_PIN_10
#define AD7606_BUSY_GPIO_PORT         GPIOF
#define AD7606_RST_PIN                GPIO_PIN_9
#define AD7606_RST_GPIO_PORT          GPIOF
#define AD7606_CV_PIN                 GPIO_PIN_1
#define AD7606_CV_GPIO_PORT           GPIOC

#define AD7606_OS0_GPIO_PORT          GPIOE
#define AD7606_OS0_PIN                GPIO_PIN_3
#define AD7606_OS1_GPIO_PORT          GPIOB
#define AD7606_OS1_PIN                GPIO_PIN_6
#define AD7606_OS2_GPIO_PORT          GPIOA
#define AD7606_OS2_PIN                GPIO_PIN_3
#define AD7606_RANG_GPIO_PORT         GPIOB
#define AD7606_RANG_PIN               GPIO_PIN_7

// Controlling definition of AD7606
#define AD7606_CS_HIGH					      	HAL_GPIO_WritePin(AD7606_CS_GPIO_PORT, AD7606_CS_PIN, GPIO_PIN_SET)
#define AD7606_CS_LOW							      HAL_GPIO_WritePin(AD7606_CS_GPIO_PORT, AD7606_CS_PIN, GPIO_PIN_RESET)
#define AD7606_RST_HIGH					      	HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_PIN, GPIO_PIN_SET)
#define AD7606_RST_LOW					      	HAL_GPIO_WritePin(AD7606_RST_GPIO_PORT, AD7606_RST_PIN, GPIO_PIN_RESET)
#define AD7606_CV_HIGH						      HAL_GPIO_WritePin(AD7606_CV_GPIO_PORT, AD7606_CV_PIN, GPIO_PIN_SET)
#define AD7606_CV_LOW						      	HAL_GPIO_WritePin(AD7606_CV_GPIO_PORT, AD7606_CV_PIN, GPIO_PIN_RESET)

#define AD7606_OS_RATIO_0             0
#define AD7606_OS_RATIO_2             2
#define AD7606_OS_RATIO_4             4
#define AD7606_OS_RATIO_8             8
#define AD7606_OS_RATIO_16            16
#define AD7606_OS_RATIO_32            32
#define AD7606_OS_RATIO_64            64

#define AD7606_RANG_5V                0
#define AD7606_RANG_10V               1

#define AD7606_OS0_HIGH                 HAL_GPIO_WritePin(AD7606_OS0_GPIO_PORT, AD7606_OS0_PIN, GPIO_PIN_SET);
#define AD7606_OS0_LOW                  HAL_GPIO_WritePin(AD7606_OS0_GPIO_PORT, AD7606_OS0_PIN, GPIO_PIN_RESET);
#define AD7606_OS1_HIGH                 HAL_GPIO_WritePin(AD7606_OS1_GPIO_PORT, AD7606_OS1_PIN, GPIO_PIN_SET);
#define AD7606_OS1_LOW                  HAL_GPIO_WritePin(AD7606_OS1_GPIO_PORT, AD7606_OS1_PIN, GPIO_PIN_RESET);
#define AD7606_OS2_HIGH                 HAL_GPIO_WritePin(AD7606_OS2_GPIO_PORT, AD7606_OS2_PIN, GPIO_PIN_SET);
#define AD7606_OS2_LOW                  HAL_GPIO_WritePin(AD7606_OS2_GPIO_PORT, AD7606_OS2_PIN, GPIO_PIN_RESET);

#define AD7606_RANG_HIGH                HAL_GPIO_WritePin(AD7606_RANG_GPIO_PORT, AD7606_RANG_PIN, GPIO_PIN_SET);
#define AD7606_RANG_LOW                 HAL_GPIO_WritePin(AD7606_RANG_GPIO_PORT, AD7606_RANG_PIN, GPIO_PIN_RESET);

typedef struct
{
  SPI_HandleTypeDef*      hspi;
  int16_t                 rawData[8];
}ADCHandle;

void AD7606_Init(uint8_t AD7606_RANGE, uint8_t AD7606_OVER_SAMPLING);

void ADC_DataRequest(void);

void ADC_Read(int16_t *data);

extern ADCHandle  hADC;
#endif
