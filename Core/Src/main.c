#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"
#include "usb_device.h"
#include "system_periphrals.h"
#include "usbd_cdc_if.h"
#include "usb.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"
#include "sdram.h"
#include "touch_800x480.h"

osThreadId_t AK10_CalibrationnTaskHandle;
const osThreadAttr_t AK10_Calibration_attributes = {
  .name = "AK10_Calibration",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

void SystemClock_Config(void);
void AK10Calibration_Task(void *argument);
void LCD_Task(void *argument);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  SystemPeriphral_Init();
  USB_Init();
  LTDC_Init();
  Touch_Init();
  
  osKernelInitialize();
  AK10_CalibrationnTaskHandle = osThreadNew(AK10Calibration_Task, NULL, &AK10_Calibration_attributes);
  LCDTaskHandle = osThreadNew(LCD_Task, NULL, &LCDTask_attributes);
  osKernelStart();
  while (1){}
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3);
}

void AK10Calibration_Task(void *argument)
{
  for(;;)
  {
    GPIO_Digital_Filtered_Input(&hButtonOnboardKey, 30);
    LED_Blink(&hLEDBlue, 2);
//    CDC_Transmit_HS((uint8_t*)usbtxtest, 6);
    osDelay(10);
  }
}

void LCD_Task(void *argument)
{
  for(;;)
  {
    LED_Blink(&hLEDYellowGreen, 15);
    osDelay(10);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}
