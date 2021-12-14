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
#include "os_threads.h"

void SystemClock_Config(void);

/////for UI testing/////
ButtonHandle hButtonDataLog;
ButtonHandle hButtonDataLogEnd;
uint8_t buttoncount1;
uint8_t buttoncount2;
JoystickHandle hJoystickTest;

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
  OSThreads_Init();
  
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
    if (GPIO_Digital_Filtered_Input(&hButtonOnboardKey, 30))
    {
    }
    LED_Blink(&hLEDBlue, 2);
    AK10_9_DataLog_Manager(&hAKMotorLeftHip);

    osDelay(10);
  }
}

void LCD_Task(void *argument)
{
  hButtonDataLog = Button_Create(250, 50, 200, 50, "Data Log Start", LIGHT_MAGENTA, LCD_RED);
  hButtonDataLogEnd = Button_Create(50, 300, 100, 100, "Data Log End", LCD_GREEN, LCD_RED);
  hJoystickTest = Joystick_Create(250, 600, 100, "joystick");
  for(;;)
  {
    Touch_Scan();	// 触摸扫描
    ButtonScan(&hButtonDataLog);
    ButtonScan(&hButtonDataLogEnd);
    if (touchInfo.flag)
    {
      
    }
    else
    {
    }
    
    osDelay(10);
    if (ifButtonPressed(&hButtonDataLog))
    {
      buttoncount1++;
      LCD_DisplayNumber(300, 100, buttoncount1, 1);
      USB_DataLogStart();
    }
    if (ifButtonPressed(&hButtonDataLogEnd))
    {
      buttoncount2++;
      LCD_DisplayNumber(300, 400, buttoncount2, 1);
      USB_DataLogEnd();
    }
    
    ButtonRefresh(&hButtonDataLogEnd);
    ButtonRefresh(&hButtonDataLog);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}
