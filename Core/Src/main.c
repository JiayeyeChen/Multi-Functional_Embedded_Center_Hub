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
#include "lktech_mg_motor.h"
#include "xiaomi_cybergear.h"
#include "bldc_actuators_testing.h"
#include "cui_amt222b_encoder.h"
#include "serial_protocol.h"
#include "opticalencoder_acceleration_test.h"

void SystemClock_Config(void);


/* For exoskeleton motor test */
uint32_t datalogTimeStamp;
////////////////////////////////


int main(void)
{
	
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  
  SystemPeriphral_Init();
  USB_Init(sizeof(dataSlots_Exoskeleton_Common)/4);
  
	MX_FMC_Init();
  UI_Init();
  
//	DWT_Delay_Init();
	hSerial = SERIALPROTOCOL_Create(&huart6);
	SERIALPROTOCOL_SetNewDatalogSlotLength(&hSerial, 3);
	SERIALPROTOCOL_SetNewDatalogSlot(&hSerial, datalog_slot_cui);
	SERIALPROTOCOL_SetNewDatalogSendLabelFunction(&hSerial, OPTICALENCODERTEST_SetDatalogLabel);
	SERIALPROTOCOL_EnableCommunication(&hSerial);
	hCUIEncoder = CUI_AMT222b_Create(&hspi4, SPI_CS_GPIO_Port, SPI_CS_Pin, 0.0f);
	LowPassFilter_Init(&hFilterSpeed, 10, 0.001f);
	LowPassFilter_Init(&hFilterAcc, 10, 0.001f);
	pre_speed = 0.0f;
	cur_speed = 0.0f;
	pre_acc = 0.0f;
	cur_acc = 0.0f;
	pre_angle = 0.0f;
	cur_angle = 0.0f;
////  EXOSKELETON_MotorInit();
//  LKTECH_MG_Init(&hLKTECH, &hcan2, 1, 36.0, 1.0f);
  
  AD7606_Init(AD7606_RANG_10V, AD7606_OS_RATIO_4);
	
////  EXOSKELETON_Init();
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

void Main_Task(void *argument)
{
  datalogTimeStamp = HAL_GetTick();
  for(;;)
  {
    /* Lin Kong Ke Ji MG */
    if (hUI.curPage == &UIPage_LinKongKeJi_Testing)
    {
      if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL)
        LETECH_MG_SpeedControl(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->velocityControlSet.f);
      else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL)
      {
//        if (ifVelDampingCompensation)
//        {
//          hLKTechTestingMotorPtr->currentControlSet.f += velComFactor * hLKTechTestingMotorPtr->speedDeg.f;
//        }
        LETECH_MG_CurrentControl(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->currentControlSet.f);
      }
      else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT)
        LETECH_MG_PositionControl6Increment(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->positionControlIncrementSet.f, 600.0f);
      
//      dataSlots_LKTECH_MG_MotorTest[0].f = hLKTechTestingMotorPtr->angle.f;
//      dataSlots_LKTECH_MG_MotorTest[1].f = hLKTechTestingMotorPtr->speedDeg.f;
//      dataSlots_LKTECH_MG_MotorTest[2].f = hLKTechTestingMotorPtr->torque.f;
//      dataSlots_LKTECH_MG_MotorTest[3].f = hLKTechTestingMotorPtr->temperature.f;
//      hUSB.ifNewDataLogPiece2Send = 1;
//      USB_DataLogManager(LKTECH_MotorTest_Set_Datalog_Label, dataSlots_LKTECH_MG_MotorTest);
    }
    
    
    
    
    
    
    
//    Foshan4DOFExoskeletonTMotor_CenterControl();
////////////////    if (hUI.curPage == &UIPage_FoshanHipExoskeleton)
////////////////      FOSHANHIPEXOSKELETON_CentreControl();
////////////////    
////////////////		if (hUI.curPage == &UIPage_Foshan4DOFExoskeletonTMotor)
////////////////			Foshan4DOFExoskeletonTMotor_CenterControl();
////////////////		else if (hUI.curPage == &UIPage_Foshan4DOFExoskeletonLKMotor)
////////////////		{
////////////////		}

////////////    /* Proprioception lower limb exoskeleton control */
////////////    if (hUI.task == UI_MAIN_TASK_LOWER_LIMB_EXOSKELETON)
////////////    {
////////////      EXOSKELETON_CentreControl();
////////////      if (hUI.curPage == &UIPage_LowerLimb_Exoskeleton)
////////////        EXOSKELETON_CommonDatalogManager();
////////////    }
////////////    
////////////    /* Tmotor acceleration observer */
////////////    if (hUI.task == UI_MAIN_TASK_TMOTOR_ACCELERATION_OBSERVER)
////////////    {
////////////      if (ifMotorProfilingStarted)
////////////        AK10_9_MotorProfiling_Function1_Half_Sin(&hAKMotorRightHip, tmotorProfilingSinWaveFrequency);
////////////    }
////////////    
////////////    /* Tmotor manual control */
////////////    if (hUI.task == UI_MAIN_TASK_AK10_9_MANUAL_CONTROL)
////////////    {
////////////      if (ifManualControlStarted)
////////////      {
////////////        if (hUI.curPage == &UIPage_AK10_9_ManualControlCubeMarsFWServoMode)
////////////        {
////////////          if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_POSITION)
////////////            AK10_9_ServoMode_PositionControl(hMotorPtrManualControl, manualControlValue_pos);
////////////          else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_CURRENT)
////////////            AK10_9_ServoMode_CurrentControl(hMotorPtrManualControl, manualControlValue_cur);
////////////          else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_VELOCITY)
////////////            AK10_9_ServoMode_VelocityControl(hMotorPtrManualControl, manualControlValue_vel);
////////////        }
////////////        else if (hUI.curPage == &UIPage_AK10_9_ManualControlCubeMarsFWMITMode)
////////////        {
////////////          AK10_9_CubeMarsFW_MITMode_ContinuousControl_Deg(hMotorPtrManualControl, \
////////////                                                          manualControlValue_pos, manualControlValue_vel, \
////////////                                                          manualControlValue_kp, manualControlValue_kd, manualControlValue_cur);
////////////          AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(hMotorPtrManualControl, \
////////////                                                             30.0f, 180.0f, 1.0f, 100.0f, 0.5f, 0.001f);
////////////        }
////////////      }
////////////    }
////////////    
////////////    /* Customized IMU Monitor */
////////////    if (hUI.task == UI_MAIN_TASK_IMU_MONITOR)
////////////    {
////////////      if (ifIMUFeedbackStarted)
////////////        EXOSKELETON_SetBNO055Mode_ACC_Only(&hIMUTorso);
////////////    }
////////////    
////////////    /* Exoskeleton motor test */
////////////    if (hUI.task == UI_MAIN_TASK_EXOSKELETON_MOTOR_TEST)
////////////    {
////////////      if (ifMotorProfilingStartedExoskeletonMotorTest)
////////////      {
//////////////////        exoskeletonMotorTestHipGaitAngleDeg = rad2deg * EXOSKELETON_HipGait1(((float)(HAL_GetTick() - exoskeletonMotorTestTimeDifference))/1000.0f);
//////////////////        AK10_9_CubeMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, exoskeletonMotorTestHipGaitAngleDeg, \
//////////////////                                                                    0.0f, 499.0f, 3.0f, 0.0f);
////////////        AK10_9_CubeMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, 0.0f, 0.0f, 0.0f, 0.0f, -35.0f / hAKMotorRightHip.kt);
////////////      }
////////////      AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
////////////                                                             300.0f, 180.0f, 5.0f, 100.0f, 0.5f, 0.001f);
////////////      dataSlots_Exoskeleton_Motor_Durability_Test[0].f = hAKMotorRightHip.realPositionOffseted.f;
////////////      dataSlots_Exoskeleton_Motor_Durability_Test[1].f = hAKMotorRightHip.realVelocityPresent.f;
////////////      dataSlots_Exoskeleton_Motor_Durability_Test[2].f = hAKMotorRightHip.realCurrent.f;
////////////      dataSlots_Exoskeleton_Motor_Durability_Test[3].f = hAKMotorRightHip.temperature;
////////////      if (HAL_GetTick() - datalogTimeStamp > 100)
////////////      {
////////////        hUSB.ifNewDataLogPiece2Send = 1;
////////////        datalogTimeStamp = HAL_GetTick();
////////////      }
////////////      USB_DataLogManager(EXOSKELETON_Motor_Durability_Test_Set_Datalog_Label, dataSlots_Exoskeleton_Motor_Durability_Test);
////////////    }

////////////    AK10_9_CubeMarsFW_MotorStatusMonitor(&hAKMotorRightKnee, 100);
////////////    AK10_9_CubeMarsFW_MotorStatusMonitor(&hAKMotorRightHip, 100);
    
    osDelay(10);
  }
}

void MotorTesting_Task(void *argument)
{
	SERIALPROTOCOL_EnableCommunication(&hSerial);
//	SERIALPROTOCOL_DatalogInitiateStart(&hSerial);
	for(;;)
  {
//		CUI_AMT222b_Get_Angle(&hCUIEncoder);
		pre_angle = cur_angle;
		cur_angle = hCUIEncoder.angleDeg;
		
		pre_speed = hFilterSpeed.output.f;
		LowPassFilter_Update(&hFilterSpeed, (cur_angle - pre_angle) / 0.001f);
		cur_speed = hFilterSpeed.output.f;
		
		LowPassFilter_Update(&hFilterAcc, (cur_speed - pre_speed) / 0.001f);
		
		datalog_slot_cui[0].f = hCUIEncoder.angleDeg;
		datalog_slot_cui[1].f = cur_speed;
		datalog_slot_cui[2].f = hFilterAcc.output.f;
		
		hSerial.ifNewDatalogPiece2Send = 1;
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
//    ADC_DataRequest();
		SERIALPROTOCOL_DatalogManager(&hSerial);
    osDelay(10);
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

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	SERIALPROTOCOL_ReceiveCargoUARTIdleITCallback(&hSerial);
}
