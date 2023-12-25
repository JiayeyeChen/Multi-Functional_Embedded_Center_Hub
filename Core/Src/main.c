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
#include "foshan_hip_exoskeleton.h"
#include "xiaomi_cybergear.h"
#include "foshan_4dof_exoskeleton_tmotor.h"
#include "bldc_actuators_testing.h"
#include "hwt605-can-inclinometer.h"
#include "serial_protocol.h"
#include "embedded_multimeter.h"



/* For exoskeleton motor test */
uint32_t datalogTimeStamp;
////////////////////////////////

SerialProtocolHandle hSerial;
union FloatUInt8 dataSlots_HWT605[7];
float standstillAccX, standstillAccY, standstillAccZ, hipAngle;

EmbeddedMultimeterHandle hMeter;

void SendDatalogLabels_HWT605(void)
{
	SERIALPROTOCOL_SendDataSlotLabel(&hSerial, "7", "AccX", "AccY", "AccZ", "AccXOffset", "AccYOffset", "AccZOffset", "Hip Angle");
}

int main(void)
{
	
  HAL_Init();

  SystemClock_Config();

  MX_FATFS_Init();
  
  SystemPeriphral_Init();
  USB_Init(sizeof(dataSlots_Exoskeleton_Common)/4);
  
	MX_FMC_Init();
  UI_Init();
  
  himu = HWT605_Create(&hcan2, 0x50, 0.9821f, 0.027f, -0.0064f, -0.0166f, 1.0097f, -0.0058f, \
	-0.0062f, 0.0025f, 1.0063f, 0.0598f, 0.0614f, 0.0654f);
//	hMeter = EMBEDDEDMULTIMETER_Create(&huart3, 0x01);
	
	
	hSerial = SERIALPROTOCOL_Create(&huart6);
	SERIALPROTOCOL_EnableCommunication(&hSerial);
	
	SERIALPROTOCOL_SetNewDatalogSlotLength(&hSerial, sizeof(dataSlots_HWT605)/4);
	SERIALPROTOCOL_SetNewDatalogSlot(&hSerial, dataSlots_HWT605);
	SERIALPROTOCOL_SetNewDatalogSendLabelFunction(&hSerial, SendDatalogLabels_HWT605);
	
	
	
  CAN_ConfigureFilters();
	
  HAL_Delay(50);
  osKernelInitialize();
  OSThreads_Init();
  
  osKernelStart();
  while (1){}
}



void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if (huart == hSerial.huart)
		SERIALPROTOCOL_ReceiveCargoUARTIdleITCallback(&hSerial);
	else if (huart == hMeter.huart)
		EMBEDDEDMULTIMETER_GetData(&hMeter);
}


void Main_Task(void *argument)
{
  datalogTimeStamp = HAL_GetTick();
  
  for(;;)
  {
		
//		EMBEDDEDMULTIMETER_ReadAllRequest(&hMeter);
		
		
		dataSlots_HWT605[0].f = himu.AccX.f;
		dataSlots_HWT605[1].f = himu.AccY.f;
		dataSlots_HWT605[2].f = himu.AccZ.f;
		dataSlots_HWT605[3].f = standstillAccX;
		dataSlots_HWT605[4].f = standstillAccY;
		dataSlots_HWT605[5].f = standstillAccZ;
		hipAngle = acosf((himu.AccX.f * standstillAccX + himu.AccY.f * standstillAccY + himu.AccZ.f * standstillAccZ) / \
								(sqrtf(himu.AccX.f*himu.AccX.f + himu.AccY.f*himu.AccY.f + himu.AccZ.f*himu.AccZ.f) * \
								sqrtf(standstillAccX*standstillAccX + standstillAccY*standstillAccY + standstillAccZ*standstillAccZ))) * rad2deg;
		dataSlots_HWT605[6].f = hipAngle;
		hSerial.ifNewDatalogPiece2Send = 1;
		SERIALPROTOCOL_DatalogManager(&hSerial);
		
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
    
    osDelay(5);
  }
}

void MotorTesting_Task(void *argument)
{
	for(;;)
  {
		/* Lin Kong Ke Ji MG */
//    if (hUI.curPage == &UIPage_LinKongKeJi_Testing)
//    {
//      if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL)
//        LETECH_MG_SpeedControl(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->velocityControlSet.f);
//      else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL)
//      {
//        LETECH_MG_CurrentControl(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->currentControlSet.f + velComFactor * hLKTechTestingMotorPtr->speedDeg.f);
//      }
//      else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT)
//        LETECH_MG_PositionControl6Increment(hLKTechTestingMotorPtr, hLKTechTestingMotorPtr->positionControlIncrementSet.f, 600.0f);
//      
////      dataSlots_LKTECH_MG_MotorTest[0].f = hLKTechTestingMotorPtr->angle.f;
////      dataSlots_LKTECH_MG_MotorTest[1].f = hLKTechTestingMotorPtr->speedDeg.f;
////      dataSlots_LKTECH_MG_MotorTest[2].f = hLKTechTestingMotorPtr->torque.f;
////      dataSlots_LKTECH_MG_MotorTest[3].f = hLKTechTestingMotorPtr->temperature.f;
////      hUSB.ifNewDataLogPiece2Send = 1;
////      USB_DataLogManager(LKTECH_MotorTest_Set_Datalog_Label, dataSlots_LKTECH_MG_MotorTest);
//    }
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
