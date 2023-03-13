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
#include "BenMoKeJi_M15.h"
#include "lktech_mg_motor.h"
#include "bldc_actuators_testing.h"
#include "mrdoor.h"

void SystemClock_Config(void);

int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  
  SystemPeriphral_Init();
  USB_Init(sizeof(dataSlots_Exoskeleton_Common)/4);
  
	MX_FMC_Init();
  UI_Init();
  
  EXOSKELETON_MotorInit();
	MRDOOR_MotorInit();

  AD7606_Init(AD7606_RANG_10V, AD7606_OS_RATIO_4);
//  BENMOKEJI_M15_Init(&hBENMOKEJI, &hcan2, 2);
	LKTECH_MG_Init(&hLKTECH, &hcan2, 1, 36.0f);
  EXOSKELETON_Init();
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
    if (hExoskeleton.mainTask != EXOSKELETON_MAIN_TASK_SYSTEM_ID)
      EXOSKELETON_CommonDatalogManager();

    if (ifMotorProfilingStarted)
      AK10_9_MotorProfiling_Function1_Half_Sin(&hAKMotorRightHip, tmotorProfilingSinWaveFrequency);
    
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
    }
    
    if (ifIMUFeedbackStarted)
      EXOSKELETON_SetBNO055Mode_ACC_Only(&hIMUTorso);
    
    AK10_9_CubeMarsFW_MotorStatusMonitor(&hAKMotorRightKnee, 100);
    AK10_9_CubeMarsFW_MotorStatusMonitor(&hAKMotorRightHip, 100);
    
    osDelay(1);
  }
}

void MotorTesting_Task(void *argument)
{
	for(;;)
  {
    /* Ben Mo Ke Ji M15 */
    if (hUI.curPage == &UIPage_BenMoKeJiM15_Testing)
    {
      if (hBENMOKEJI.mode == BENMOKEJI_MODE_POSITION)
        BENMODEJI_M15_PositionControlSingleMotor(&hBENMOKEJI, hBENMOKEJI.positionSetDeg.f);
      else if (hBENMOKEJI.mode == BENMOKEJI_MODE_VELOCITY)
        BENMODEJI_M15_VelocityControlDegSingleMotor(&hBENMOKEJI, hBENMOKEJI.speedSetDeg.f);
      else if (hBENMOKEJI.mode == BENMOKEJI_MODE_CURRENT)
        BENMODEJI_M15_CurrentControlSingleMotor(&hBENMOKEJI, hBENMOKEJI.currentSet.f);
    }
		
    //////////////////////
		/* Lin Kong Ke Ji MG */
		if (hLKTECH.task == LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL)
			LETECH_MG_SpeedControl(&hLKTECH, hLKTECH.velocityControlSet.f);
		else if (hLKTECH.task == LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL)
			LETECH_MG_CurrentControl(&hLKTECH, hLKTECH.currentControlSet.f);
		else if (hLKTECH.task == LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT)
			LETECH_MG_PositionControl6Increment(&hLKTECH, hLKTECH.positionControlIncrementSet.f, 600.0f);
    
    osDelay(10);
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
//    ADC_DataRequest();
    osDelay(1000);
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
