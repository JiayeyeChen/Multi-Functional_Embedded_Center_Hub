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
#include "can_bus.h"
#include "my_math.h"
#include "ak10-9_v2_testing.h"
#include <math.h>
#include "user_interface.h"

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

/////for testing/////
float sinPosTest = 0.0f;
double sinIncre = 1.0f;
double step = 0.0005f;
/////////////////////
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  
  SystemPeriphral_Init();
  USB_Init();
  
	MX_FMC_Init();
  UI_Init();
  MotorInit();
  
  hAKMotorLeftHip.rxFilter = ConfigCANFilter_EXT_ID_32BitIDListMode(&hcan1, 0, CAN_FILTER_FIFO0, CAN_ID_EXT, CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP, 0);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_Start(&hcan2);
  
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

	uint16_t LCD_PLLSAIN = 0;		//	用于倍频的PLLSAIN参数，可取范围为50~432
	uint8_t  LCD_PLLSAIR = 3;		//	用于分频的PLLSAIR参数，可取范围为2~7
	uint8_t  LCD_CLKDIV	= 2;		//	LCD时钟分频参数，默认设置为2分频
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
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
  
  
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
	// LCD_CLK = LCD_PLLSAIN / LCD_PLLSAIR / RCC_PLLSAIDIVR_2
	LCD_PLLSAIN = LCD_CLK * LCD_PLLSAIR * LCD_CLKDIV;	//	根据需要使用的LCD时钟计算PLLSAIN参数，可取范围为50~432
	PeriphClkInitStruct.PLLSAI.PLLSAIN 	= LCD_PLLSAIN;			// 设置 PLLSAIN
	PeriphClkInitStruct.PLLSAI.PLLSAIR 	= LCD_PLLSAIR;			// 设置 PLLSAIR，这里取值为3
	PeriphClkInitStruct.PLLSAIDivR 		= RCC_PLLSAIDIVR_4;	// 为了方便计算，这里使用2分频，所以 LCD_CLKDIV 定义为2
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

/////////////////////////////////////////////////////////////////////////////

//	uint16_t LCD_PLLSAIN = 0;		//	用于倍频的PLLSAIN参数，可取范围为50~432
//	uint8_t  LCD_PLLSAIR = 3;		//	用于分频的PLLSAIR参数，可取范围为2~7
//	uint8_t  LCD_CLKDIV	= 2;		//	LCD时钟分频参数，默认设置为2分频

//	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
//	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
//	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

//	/** Configure the main internal regulator output voltage 
//	*/
//	__HAL_RCC_PWR_CLK_ENABLE();
//	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
//	/** Initializes the CPU, AHB and APB busses clocks 
//	*/
//	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
//	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
//	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
//	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
//	RCC_OscInitStruct.PLL.PLLM = 25;
//	RCC_OscInitStruct.PLL.PLLN = 360;
//	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
//	RCC_OscInitStruct.PLL.PLLQ = 4;
//	HAL_RCC_OscConfig(&RCC_OscInitStruct);
//	/** Activate the Over-Drive mode 
//	*/
//	HAL_PWREx_EnableOverDrive();
//	/** Initializes the CPU, AHB and APB busses clocks 
//	*/
//	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
//										|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
//	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
//	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
//	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
//	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

//	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);
//	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
//	
//	// LCD_CLK = LCD_PLLSAIN / LCD_PLLSAIR / RCC_PLLSAIDIVR_2
//	LCD_PLLSAIN = LCD_CLK * LCD_PLLSAIR * LCD_CLKDIV;	//	根据需要使用的LCD时钟计算PLLSAIN参数，可取范围为50~432
//	
//	PeriphClkInitStruct.PLLSAI.PLLSAIN 	= LCD_PLLSAIN;			// 设置 PLLSAIN
//	PeriphClkInitStruct.PLLSAI.PLLSAIR 	= LCD_PLLSAIR;			// 设置 PLLSAIR，这里取值为3
//	PeriphClkInitStruct.PLLSAIDivR 		= RCC_PLLSAIDIVR_2;	// 为了方便计算，这里使用2分频，所以 LCD_CLKDIV 定义为2
//	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);
}

void AK10Calibration_Task(void *argument)
{
  AK10_9_ServoMode_Zeroing(&hAKMotorLeftHip);
  for(;;)
  {
    sinPosTest = (float)sin(sinIncre * step) * 3600.0f;
    AK10_9_ServoMode_PositionSpeedControl(&hAKMotorLeftHip, sinPosTest, 1000.0f, 0x0FFF);
    sinIncre+=1.0f;
    if (GPIO_Digital_Filtered_Input(&hButtonOnboardKey, 30))
    {
//      AK10_9_ServoMode_PositionSpeedControl(&hAKMotorLeftHip, 2000.0f, 1000.0f, 0x0FFF);
      LCD_SetLayer(0);
      LCD_SetColor(LIGHT_MAGENTA);
      LCD_FillCircle(100, 100, 80);
      step+=1.0f;
    }
    LED_Blink(&hLEDBlue, 2);
    
    osDelay(10);
  }
}

void LCD_Task(void *argument)
{
  LCD_SetLayer(0);
  LCD_SetColor(LIGHT_MAGENTA);
  LCD_FillCircle(100, 100, 50);
  for(;;)
  {
    Touch_Scan();	// 触摸扫描
    if (touchInfo.flag)
    {
      LED_Blink(&hLEDYellowGreen, 10);
      LCD_SetLayer(1);
      LCD_SetColor(LCD_BLACK);
      LCD_FillCircle(100, 100, 20);
    }
    else
      LED_Off(&hLEDYellowGreen);
    
    osDelay(100);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}
