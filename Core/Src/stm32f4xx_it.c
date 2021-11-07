#include "main.h"
#include "stm32f4xx_it.h"

extern PCD_HandleTypeDef hpcd_USB_OTG_HS;
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern DAC_HandleTypeDef hdac;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_sdio_rx;
extern DMA_HandleTypeDef hdma_sdio_tx;
extern SD_HandleTypeDef hsd;
extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;
extern DMA_HandleTypeDef hdma_spi5_rx;
extern DMA_HandleTypeDef hdma_spi5_tx;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi4;
extern SPI_HandleTypeDef hspi5;
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim3;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim1;

//for testing//

///////////////
void NMI_Handler(void)
{
  while (1)
  {
  }
}

void HardFault_Handler(void)
{
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  while (1)
  {
  }
}

void DebugMon_Handler(void)
{
}

void EXTI2_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
}

void EXTI3_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void EXTI4_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
}

void DMA1_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c1_rx);
}

void DMA1_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
}

void DMA1_Stream2_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart3_tx);
}

void DMA1_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
}

void DMA1_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
}

void DMA1_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
}

void CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void CAN1_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan1);
}

void EXTI9_5_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
}

void TIM1_UP_TIM10_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim3);
}

void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&hi2c1);
}

void I2C1_ER_IRQHandler(void)
{
  HAL_I2C_ER_IRQHandler(&hi2c1);
}

void SPI1_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi1);
}

void USART2_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart2);
}

void USART3_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart3);
}

void EXTI15_10_IRQHandler(void)
{
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
}

void TIM8_UP_TIM13_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim13);
}

void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim14);
}

void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_i2c1_tx);
}

void SDIO_IRQHandler(void)
{
  HAL_SD_IRQHandler(&hsd);
}

void TIM5_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim5);
}

void UART4_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart4);
}

void TIM6_DAC_IRQHandler(void)
{
  HAL_DAC_IRQHandler(&hdac);
}

void DMA2_Stream0_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi4_rx);
}

void DMA2_Stream1_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi4_tx);
}

void DMA2_Stream2_IRQHandler(void)
{
  
}

void DMA2_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sdio_rx);
}

void DMA2_Stream4_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi5_tx);
}

void CAN2_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_RX1_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

void CAN2_SCE_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan2);
}

void DMA2_Stream5_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_spi5_rx);
}

void DMA2_Stream6_IRQHandler(void)
{
  HAL_DMA_IRQHandler(&hdma_sdio_tx);
}

void DMA2_Stream7_IRQHandler(void)
{
  
}

void USART6_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart6);
}

void OTG_HS_EP1_OUT_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}

void OTG_HS_EP1_IN_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}

void OTG_HS_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
}

void SPI4_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi4);
}

void SPI5_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi5);
}

void SPI6_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi6);
}
