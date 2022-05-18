#include "adc.h"

SPI_HandleTypeDef hspi5;
ADCHandle         hADC;

void AD7606_Init(uint8_t AD7606_RANGE, uint8_t AD7606_OVER_SAMPLING)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
  /* SPI1 parameter configuration*/
	 __HAL_RCC_SPI5_CLK_ENABLE();
	
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  HAL_SPI_Init(&hspi5);
	
	/* AD7606 GPIO configuration*/
	GPIO_InitStruct.Pin = AD7606_CS_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_CS_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7606_CLK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
	HAL_GPIO_Init(AD7606_CLK_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = AD7606_MISO_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI5;
	HAL_GPIO_Init(AD7606_MISO_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = AD7606_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_RST_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = AD7606_CV_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_CV_GPIO_PORT, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = AD7606_BUSY_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(AD7606_BUSY_GPIO_PORT, &GPIO_InitStruct);
  
	HAL_NVIC_SetPriority(AD7606_BUSY_EXTI_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(AD7606_BUSY_EXTI_IRQn);
  
  GPIO_InitStruct.Pin = AD7606_OS0_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_OS0_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7606_OS1_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_OS1_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7606_OS2_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_OS2_GPIO_PORT, &GPIO_InitStruct);
  
  GPIO_InitStruct.Pin = AD7606_RANG_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(AD7606_RANG_GPIO_PORT, &GPIO_InitStruct);

  hADC.hspi = &hspi5;
  
  switch (AD7606_OVER_SAMPLING)
  {
  	case AD7606_OS_RATIO_0:
      AD7606_OS0_LOW
      AD7606_OS1_LOW
      AD7606_OS2_LOW
  		break;
  	case AD7606_OS_RATIO_2:
      AD7606_OS0_HIGH
      AD7606_OS1_LOW
      AD7606_OS2_LOW
  		break;
    case AD7606_OS_RATIO_4:
      AD7606_OS0_LOW
      AD7606_OS1_HIGH
      AD7606_OS2_LOW
  		break;
    case AD7606_OS_RATIO_8:
      AD7606_OS0_HIGH
      AD7606_OS1_HIGH
      AD7606_OS2_LOW
  		break;
    case AD7606_OS_RATIO_16:
      AD7606_OS0_LOW
      AD7606_OS1_LOW
      AD7606_OS2_HIGH
  		break;
    case AD7606_OS_RATIO_32:
      AD7606_OS0_HIGH
      AD7606_OS1_LOW
      AD7606_OS2_HIGH
  		break;
    case AD7606_OS_RATIO_64:
      AD7606_OS0_LOW
      AD7606_OS1_HIGH
      AD7606_OS2_HIGH
  		break;
  	default:
  		break;
  }
	
  switch (AD7606_RANGE)
  {
  	case AD7606_RANG_5V:
      AD7606_RANG_LOW
  		break;
  	case AD7606_RANG_10V:
      AD7606_RANG_HIGH
  		break;
  	default:
  		break;
  }
  
	AD7606_CS_HIGH;
	AD7606_CV_LOW;
	
	// Reset AD7606
	AD7606_RST_LOW;
	HAL_Delay(10);
	AD7606_RST_HIGH;
	HAL_Delay(10);
	AD7606_RST_LOW;
}

void ADC_DataRequest(void)
{
	AD7606_CV_LOW;
	AD7606_CV_HIGH;
}

void ADC_Read(int16_t *data)
{
	AD7606_CS_LOW;
	HAL_SPI_Receive(&hspi5, (uint8_t *)data, 8, 2);
	AD7606_CS_HIGH;
}
