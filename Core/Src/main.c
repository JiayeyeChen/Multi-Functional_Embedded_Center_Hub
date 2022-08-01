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
#include "adc.h"
#include "exoskeleton.h"

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  
  SystemPeriphral_Init();
  USB_Init(sizeof(dataSlots_AK10_9_Acceleration_Observer_Testing)/4);
  
	MX_FMC_Init();
  UI_Init();
  
  EXOSKELETON_MotorInit();
  AD7606_Init(AD7606_RANG_5V, AD7606_OS_RATIO_0);

  EXOSKELETON_Init();
  ENCODER_Init();
  CAN_ConfigureFilters();
  
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
  
  for(;;)
  {
    EXOSKELETON_CentreControl();
    EXOSKELETON_CommonDatalogManager();
//    AK10_9_DataLog_Manager_DM_FW(&hAKMotorRightHip, &hIMURightThigh);
    if (ifMotorProfilingStarted)
      AK10_9_MotorProfiling_Function1_Half_Sin(&hAKMotorRightHip_old, tmotorProfilingSinWaveFrequency);
    
    if (ifManualControlStarted)
    {
      if (hUI.curPage == &UIPage_AK10_9_ManualControlCubeMarsFWServoMode)
      {
        if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_POSITION)
          AK10_9_ServoMode_PositionControl(hMotorPtrManualControl, manualControlValue_pos);
        else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_CURRENT)
          AK10_9_ServoMode_CurrentControl(hMotorPtrManualControl, manualControlValue_cur);
        else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_VELOCITY)
          AK10_9_ServoMode_VelocityControl(hMotorPtrManualControl, manualControlValue_vel);
      }
      else if (hUI.curPage == &UIPage_AK10_9_ManualControlCubeMarsFWMITMode)
      {
        AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(hMotorPtrManualControl, \
                                                        manualControlValue_pos, manualControlValue_vel, \
                                                        manualControlValue_kp, manualControlValue_kd, manualControlValue_cur);
        AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(hMotorPtrManualControl, \
                                                           30.0f, 180.0f, 1.0f, 100.0f, 0.5f, 0.001f);
      }
      else if (hUI.curPage == &UIPage_AK10_9_ManualControlDMFW)
      {
        if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_MIT)
        {
          AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorRightHip, \
                                                    manualControlValue_pos, manualControlValue_vel, \
                                                    manualControlValue_kp, manualControlValue_kd, manualControlValue_cur);
          AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
                                                        30.0f, 180.0f, 1.0f, 100.0f, 0.5f, 0.001f);
        }
        else if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_VELOCITY)
          AK10_9_DMFW_VelocityControl(&hAKMotorRightHip, manualControlValue_vel);
        else if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_POSITION)
          AK10_9_DMFW_PositionVelocityControl(&hAKMotorRightHip, manualControlValue_pos, manualControlValue_vel);
      }
    }
    
    if (ifIMUFeedbackStarted)
      EXOSKELETON_SetIMUMode_ACC_Only(&hIMURightThigh);
    
    AK10_9_CubeMarsFW_MotorStatusMonitor(&hAKMotorRightKnee, 100);
    AK10_9_DMFW_MotorStatusMonitor(&hAKMotorRightHip, 100);
    
    osDelay(1);
  }
}

void UI_Task(void *argument)
{
  
  for(;;)
  {
    Touch_Scan();
    UI();
    LED_Blink(&hLEDBlue, 2);
    osDelay(50);
  }
}

void ADC_Task(void *argument)
{
  for(;;)
  {
    ADC_DataRequest();
    osDelay(1);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch(GPIO_Pin)
  {
  case AD7606_BUSY_PIN:
    ADC_ReadRawData(&hADC);
    ADC_GetVoltage(&hADC);
    break;
  
  default:
    break;
  }
}
