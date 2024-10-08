#include "user_interface.h"
#include "main.h"
#include <math.h>

UIHandle hUI;

PageHandle UIPage_Home1;
/* Common Stuff */
ButtonHandle hButtonGoBack, hButtonDataLogStart, hButtonDataLogEnd, \
             hButtonMotorProfilingStart, hButtonMotorProfilingEnd, \
             hButtonMotorZeroing, hButtonMotorStart, hButtonMotorStop, \
             hButtonStart, hButtonStop, hButtonOn, hButtonOff,\
						 hButtonPositionControl, hButtonVelocityControl, hButtonCurrentControl, \
						 hButtonMotorEnable, hButtonMotorDisable, \
             hButtonSelectSomething;
//////////////////
/* Exoskeleton User Interface */
PageHandle UIPage_LowerLimb_Exoskeleton, UIPage_LowerLimb_SystemID, UIPage_LowerLimb_GravityCompensation, \
           UIPage_LowerLimb_ParameterAdjusting, UIPage_LowerLimb_MuscularTorqueMonitor;
ButtonHandle hButtonPageExoskeletonInterface, hButtonPageSystemID, hButtonPageGravityCompensation, \
             hButtonPageExoskeletonParameterPanel, hButtonPageExoskeletonMuscularTorqueMonitor, \
             hButtonSystemIDJointMovementStart, hButtonHipMotorZeroing, \
             hButtonKneeMotorZeroing, hButtonProfilingTimeIncrease, hButtonProfilingTimeDecrease, \
             hButtonMotorEnable, hButtonMotorDisable, hButtonAugmentedControlOn, hButtonAugmentedControlOff;
             
LinearPotentialmeterHandle hPotKneeProfilingFreq, hPotKneeProfilingAmp, hPotHipProfilingFreq, hPotHipProfilingAmp, \
                           hPotGravityCompensationHipThrottle, hPotGravityCompensationKneeThrottle, \
                           hPotParameterAdjust_L1, hPotParameterAdjust_J1, hPotParameterAdjust_X1, \
                           hPotParameterAdjust_J2, hPotParameterAdjust_X2, \
                           hPotAugmentedControlHipThrottle, hPotAugmentedControlKneeThrottle;
////////////////////////////////
/* AK10-9 Manual Control */
PageHandle UIPage_AK10_9_ManualControlCubeMarsFWServoMode, UIPage_AK10_9_ManualControlCubeMarsFWMITMode;
ButtonHandle hButtonPageAK10_9ManualControl, hButtonManualControlMode, hButtonMotorSelectRightHip, \
                                             hButtonMotorSelectRightKnee, \
                                             hButtonAK10_9_ManualControlCubeMarsFWServoMode, \
                                             hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode;
LinearPotentialmeterHandle  hTMotorManualControlPot_pos, hTMotorManualControlPot_vel, \
                            hTMotorManualControlPot_cur, hTMotorManualControlPot_kp, \
                            hTMotorManualControlPot_kd;
///////////////////////////
/* BNO055 Monitor */
PageHandle UIPage_BNO055_Monitor;
ButtonHandle hButtonPageBNO055_Monitor, hButtonIMUSetModeNDOF, \
             hButtonIMUSetModeACCONLY, hButtonIMUSetModeGYROONLY;
////////////////////
/* Acceleration Observer Development*/
PageHandle UIPage_Acceleration_Observer_Project;
ButtonHandle hButtonPageAccelerationObserverProject;
//////////////////////////////////////

/* AD7606 Monitor */
PageHandle UIPage_ADC_Monitor;
ButtonHandle hButtonPageADCMonitor, hButtonResetAD7606;
////////////////////
/*Combined BNO055 and ADXL355 Customized IMU monitor*/
PageHandle UIPage_Customized_IMU_Monitor;
ButtonHandle hButtonPageCustomizedIMUMonitor, hButtonSelectHipIMU, hButtonSelectKneeIMU, \
             hButtonAveragingStart, hButtonAveragingStop;
//////////////////////////////////////////////////////

/* Lin Kong Ke Ji Testing */
PageHandle                    UIPage_LinKongKeJi_Testing;
ButtonHandle                  hButtonPageLinKongKeJiTesting, hButtonReadAngle;
ButtonHandle                  hButtonEnableVelocityDampingCompensation, hButtonDisableVelocityDampingCompensation;
JoystickHandle 								hJoyLKTECHTesting;
float                         LKTECHJoyStickValue, LKTECHBlank;
uint8_t												ifKeepReadingAngle = 0;
uint8_t                       LKTECHifMotorProfiling = 0;
LKTECH_MG_Handle*             hLKTechTestingMotorPtr;
LinearPotentialmeterHandle    hPotVelComFactor;
////////////////////////////

/* Torque Constant Calibration */
PageHandle                    UIPage_TorqueConstantCalibration;
ButtonHandle                  hButtonPageTorqueConstantCalibration, hButtonTkCalculateAverageIq;
float                         torqueConstantCalibrationMotorKp, torqueConstantCalibrationMotorKd, torqueConstantCalibrationMotorSetP;
AveragerHandle                hAverageTorqueConstantCalibration;
uint8_t                       torqueConstantCalibrationIfMotorStarted;
/////////////////////////////////

ButtonHandle Button_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, char label[],\
                           uint32_t colorUnpressed, uint32_t colorPressed)
{
  ButtonHandle hbutton;
  hbutton.pos.x = x;
  hbutton.pos.y = y;
  hbutton.pos.xLen = xLen;
  hbutton.pos.yLen = yLen;
  hbutton.colorPressed = colorPressed;
  hbutton.colorUnpressed = colorUnpressed;
  hbutton.ifNeedRefresh = 0;
  hbutton.ifPressed = 0;
  hbutton.preIfPressed = 0;

  LCD_SetLayer(0);
  LCD_SetColor(hbutton.colorUnpressed);
  LCD_FillRect(hbutton.pos.x, hbutton.pos.y, hbutton.pos.xLen, hbutton.pos.yLen);
  LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawRect(hbutton.pos.x, hbutton.pos.y, hbutton.pos.xLen, hbutton.pos.yLen);
  LCD_DisplayString(hbutton.pos.x + 10, hbutton.pos.y + 5, (char*)label);
  
  return hbutton;
}

void ButtonUpdate(ButtonHandle* hbutton)
{
  hbutton->ifPressed = 0;
  for (uint8_t i = 0; i <= 4; i++)
  {
    if ((touchInfo.xVertical[i] > hbutton->pos.x && touchInfo.xVertical[i] < (hbutton->pos.x + hbutton->pos.xLen)) &&\
        (touchInfo.yVertical[i] > hbutton->pos.y && touchInfo.yVertical[i] < (hbutton->pos.y + hbutton->pos.yLen)))
    {
      hbutton->ifPressed = 1;
      break;
    }
  }
  
  if (hbutton->ifNeedRefresh)
  {
    LCD_SetLayer(0);
    if (hbutton->ifPressed)
      LCD_SetColor(hbutton->colorPressed);
    else
      LCD_SetColor(hbutton->colorUnpressed);

    LCD_FillRect(hbutton->pos.x, hbutton->pos.y, hbutton->pos.xLen, hbutton->pos.yLen);
    hbutton->ifNeedRefresh = 0;
  }
}

uint8_t ifButtonPressed(ButtonHandle* hbutton)
{
  if (hbutton->ifPressed != hbutton->preIfPressed)
  {
    hbutton->ifNeedRefresh = 1;
    hbutton->preIfPressed = hbutton->ifPressed;
    if (hbutton->ifPressed == 0)
      return 1;
  }
  return 0;
}

LinearPotentialmeterHandle  Potentialmeter_Create(uint16_t x, uint16_t y, uint16_t xLen, \
                                                  uint16_t yLen, uint16_t sliderLen, uint16_t sliderWidth, \
                                                  uint32_t sliderColorUnpressed, uint32_t sliderColorPressed, \
                                                  uint32_t slotColor, float minVal, float maxVal, float startVal, float* ctrVal)
{
  LinearPotentialmeterHandle hpotentialmeter;
  hpotentialmeter.pos.x = x;
  hpotentialmeter.pos.y = y;
  hpotentialmeter.pos.xLen = xLen;
  hpotentialmeter.pos.yLen = yLen;
  hpotentialmeter.pos.sliderLen = sliderLen;
  hpotentialmeter.pos.sliderWidth = sliderWidth;
  hpotentialmeter.pos.ySlider = y + yLen - sliderLen - (uint16_t)(((float)yLen - (float)sliderLen)*(startVal - minVal)/(maxVal - minVal));
  hpotentialmeter.pos.xSlider = x + (xLen - sliderWidth) / 2;
  hpotentialmeter.minVal = minVal;
  hpotentialmeter.maxVal = maxVal;
  hpotentialmeter.sliderColorPressed = sliderColorPressed;
  hpotentialmeter.sliderColorUnpressed = sliderColorUnpressed;
  hpotentialmeter.ifSliderPressed = 0;
  hpotentialmeter.preIfSliderPressed = 0;
  hpotentialmeter.ifNeedRefresh = 0;
  hpotentialmeter.controlledValue = ctrVal;
  
  LCD_SetLayer(0);
  LCD_SetColor(slotColor);
  LCD_FillRect(x, y, xLen, yLen);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawRect(x, y, xLen, yLen);
  LCD_SetLayer(1);
  LCD_SetColor(sliderColorUnpressed);
  LCD_FillRect(hpotentialmeter.pos.xSlider, hpotentialmeter.pos.ySlider, hpotentialmeter.pos.sliderWidth, hpotentialmeter.pos.sliderLen);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawRect(hpotentialmeter.pos.xSlider, hpotentialmeter.pos.ySlider, hpotentialmeter.pos.sliderWidth, hpotentialmeter.pos.sliderLen);
  
  
  return hpotentialmeter;
}

void PotentialmeterUpdate(LinearPotentialmeterHandle* hpot)
{
  for (uint8_t i = 0; i <= 4; i++)
  {
    if ((touchInfo.xVertical[i] > hpot->pos.xSlider && touchInfo.xVertical[i] < (hpot->pos.xSlider + hpot->pos.sliderWidth)) &&\
        (touchInfo.yVertical[i] > hpot->pos.ySlider && touchInfo.yVertical[i] < (hpot->pos.ySlider + hpot->pos.sliderLen)))
    {
      hpot->ifSliderPressed = 1;
      if (hpot->preIfSliderPressed != hpot->ifSliderPressed)
      {
        hpot->preFingerPosY = touchInfo.yVertical[i];
        hpot->preIfSliderPressed = hpot->ifSliderPressed;
        return;
      }
      LCD_SetLayer(1);
      LCD_ClearRect(hpot->pos.xSlider - 1, hpot->pos.ySlider, hpot->pos.sliderWidth + 1, hpot->pos.sliderLen + 1);
      LCD_SetColor(hpot->sliderColorPressed);
      hpot->pos.ySlider += touchInfo.yVertical[i] - hpot->preFingerPosY;
      LIMIT_MIN_MAX(hpot->pos.ySlider, hpot->pos.y, hpot->pos.y + hpot->pos.yLen - hpot->pos.sliderLen);
      hpot->preFingerPosY = touchInfo.yVertical[i];
      LCD_FillRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
      LCD_SetColor(LCD_BLACK);
      LCD_DrawRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
      *(hpot->controlledValue) = hpot->maxVal - (hpot->maxVal - hpot->minVal) * ((float)(hpot->pos.ySlider - hpot->pos.y)/(float)(hpot->pos.yLen - hpot->pos.sliderLen));
      return;
    }
    else
    {
      hpot->ifSliderPressed = 0;
      if (hpot->preIfSliderPressed != hpot->ifSliderPressed)
      {
        hpot->preIfSliderPressed = hpot->ifSliderPressed;
        LCD_SetLayer(1);
        LCD_SetColor(hpot->sliderColorUnpressed);
        LCD_FillRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
        LCD_SetColor(LCD_BLACK);
        LCD_DrawRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
      }
    }
  }
}

void PotentialmeterSliderGoTo(LinearPotentialmeterHandle* hpot, float go_to_val)
{
  LCD_SetLayer(1);
  LCD_ClearRect(hpot->pos.xSlider - 1, hpot->pos.ySlider, hpot->pos.sliderWidth + 1, hpot->pos.sliderLen + 1);
  hpot->pos.ySlider = hpot->pos.y + (uint16_t)((float)(hpot->pos.yLen - hpot->pos.sliderLen)) * ((hpot->maxVal - go_to_val) / (hpot->maxVal - hpot->minVal));
  LCD_SetColor(hpot->sliderColorUnpressed);
  hpot->preFingerPosY = hpot->pos.ySlider;
  LCD_FillRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawRect(hpot->pos.xSlider, hpot->pos.ySlider, hpot->pos.sliderWidth, hpot->pos.sliderLen);
  *(hpot->controlledValue) = go_to_val;
}

void UI_Init(void)
{
  LTDC_Init();
  Touch_Init();
  LCD_DisplayDirection(Direction_V);
  
  /*UI handle Initialization*/
  hUI.curPage = &UIPage_Home1;
  hUI.prePage = &UIPage_Home1;
  hUI.task = UI_MAIN_TASK_NONE;
  
  /*UI Pages*/
  UIPage_Home1.ifPageInitialized = 0;
  UIPage_Home1.Page = UI_Page_Home1;
  UIPage_Home1.PageInit = UI_Page_Home1_Init;
  
  UIPage_LowerLimb_Exoskeleton.ifPageInitialized = 0;
  UIPage_LowerLimb_Exoskeleton.Page = UI_Page_LowerLimb_Exoskeleton;
  UIPage_LowerLimb_Exoskeleton.PageInit = UI_Page_LowerLimb_Exoskeleton_Init;
  
  UIPage_LowerLimb_SystemID.ifPageInitialized = 0;
  UIPage_LowerLimb_SystemID.Page = UI_Page_LowerLimb_Exoskeleton_SystemID;
  UIPage_LowerLimb_SystemID.PageInit = UI_Page_LowerLimb_Exoskeleton_SystemID_Init;
  
  UIPage_LowerLimb_GravityCompensation.ifPageInitialized = 0;
  UIPage_LowerLimb_GravityCompensation.Page = UI_Page_LowerLimb_Exoskeleton_GravityCompensation;
  UIPage_LowerLimb_GravityCompensation.PageInit = UI_Page_LowerLimb_Exoskeleton_GravityCompensation_Init;
  
  UIPage_LowerLimb_ParameterAdjusting.ifPageInitialized = 0;
  UIPage_LowerLimb_ParameterAdjusting.Page = UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting;
  UIPage_LowerLimb_ParameterAdjusting.PageInit = UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting_Init;
  
  UIPage_LowerLimb_MuscularTorqueMonitor.ifPageInitialized = 0;
  UIPage_LowerLimb_MuscularTorqueMonitor.Page = UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor;
  UIPage_LowerLimb_MuscularTorqueMonitor.PageInit = UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor_Init;
  
  UIPage_AK10_9_ManualControlCubeMarsFWServoMode.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControlCubeMarsFWServoMode.Page = UI_Page_AK10_9_ManualControlCubeMarsFWServoMode;
  UIPage_AK10_9_ManualControlCubeMarsFWServoMode.PageInit = UI_Page_AK10_9_ManualControlCubeMarsFWServoMode_Init;
  
  UIPage_AK10_9_ManualControlCubeMarsFWMITMode.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControlCubeMarsFWMITMode.Page = UI_Page_AK10_9_ManualControlCubeMarsFWMITMode;
  UIPage_AK10_9_ManualControlCubeMarsFWMITMode.PageInit = UI_Page_AK10_9_ManualControlCubeMarsFWMITMode_Init;
  
  UIPage_BNO055_Monitor.ifPageInitialized = 0;
  UIPage_BNO055_Monitor.Page = UI_Page_BNO055_Monitor;
  UIPage_BNO055_Monitor.PageInit = UI_Page_BNO055_Monitor_Init;
  
  UIPage_Acceleration_Observer_Project.ifPageInitialized = 0;
  UIPage_Acceleration_Observer_Project.Page = UI_Page_Acceleration_Observer_Project;
  UIPage_Acceleration_Observer_Project.PageInit = UI_Page_Acceleration_Observer_Project_Init;
  
  UIPage_ADC_Monitor.ifPageInitialized = 0;
  UIPage_ADC_Monitor.Page = UI_Page_ADC_Monitor;
  UIPage_ADC_Monitor.PageInit = UI_Page_ADC_Monitor_Init;
  
  UIPage_Customized_IMU_Monitor.ifPageInitialized = 0;
  UIPage_Customized_IMU_Monitor.Page = UI_Page_CustomizedIMU;
  UIPage_Customized_IMU_Monitor.PageInit = UI_Page_CustomizedIMU_Init;
  
  UIPage_LinKongKeJi_Testing.ifPageInitialized = 0;
	UIPage_LinKongKeJi_Testing.Page = UI_Page_LinKongKeJiTesting;
  UIPage_LinKongKeJi_Testing.PageInit = UI_Page_LinKongKeJiTesting_Init;
  
  UIPage_TorqueConstantCalibration.ifPageInitialized = 0;
	UIPage_TorqueConstantCalibration.Page = UI_Page_TkCalibration;
  UIPage_TorqueConstantCalibration.PageInit = UI_Page_TkCalibration_Init;

}

JoystickHandle Joystick_Create(uint16_t x, uint16_t y, uint16_t r, uint32_t background_color, \
                               uint32_t stick_color, uint16_t stick_r, uint16_t finger_detect_r, \
                               float* output_x, float* output_y, float center_val, float mag_val \
                               , float x_direction_correction, float y_direction_correction)
{
  JoystickHandle hjoy;
  hjoy.pos.x = x;
  hjoy.pos.y = y;
  hjoy.pos.r = r;
  hjoy.stickPos.x = x;
  hjoy.stickPos.y = y;
  hjoy.stickPos.r = stick_r;
  hjoy.fingerDetectR = finger_detect_r;
  hjoy.ifTouched = 0;
  hjoy.backgroundColor = background_color;
  hjoy.stickColor = stick_color;
  hjoy.outputValX = output_x;
  hjoy.outputValY = output_y;
  hjoy.outputValCenter = center_val;
  hjoy.outputValMagnitude = mag_val;
  hjoy.xDirectionCorrection = x_direction_correction;
  hjoy.yDirectionCorrection = y_direction_correction;
  LCD_SetLayer(1);
  LCD_SetColor(hjoy.stickColor);
  LCD_FillCircle(hjoy.stickPos.x, hjoy.stickPos.y, hjoy.stickPos.r);
  LCD_SetLayer(0);
  LCD_SetColor(hjoy.backgroundColor);
  LCD_FillCircle(hjoy.pos.x, hjoy.pos.y, hjoy.pos.r);
  return hjoy;
}

void JoystickUpdate(JoystickHandle* hjoy)
{
  for (uint8_t i = 0; i <= 4; i++)
  {
    float x = (float)touchInfo.xVertical[i] - (float)hjoy->pos.x;
    float y = (float)touchInfo.yVertical[i] - (float)hjoy->pos.y;
    float xy_length = sqrtf(powf(x, 2.0f) + powf(y, 2.0f));
    if (xy_length < (float)hjoy->fingerDetectR)
    {
      LCD_SetLayer(1);
      LCD_SetColor(hjoy->backgroundColor);
      LCD_FillCircle(hjoy->stickPos.x, hjoy->stickPos.y, hjoy->stickPos.r);
      hjoy->ifTouched = 1;
      if (xy_length <= fabs((float)hjoy->pos.r - (float)hjoy->stickPos.r))
      {
        hjoy->stickPos.x = touchInfo.xVertical[i];
        hjoy->stickPos.y = touchInfo.yVertical[i];
      }
      else
      {
        hjoy->stickPos.x = (uint16_t)((float)hjoy->pos.x + (((float)hjoy->pos.r - (float)hjoy->stickPos.r) / xy_length) * x);
        hjoy->stickPos.y = (uint16_t)((float)hjoy->pos.y + (((float)hjoy->pos.r - (float)hjoy->stickPos.r) / xy_length) * y);
      }
      
      break;
    }
    if (i == 4)
    {
      hjoy->ifTouched = 0;
      LCD_SetLayer(1);
      LCD_SetColor(hjoy->backgroundColor);
      LCD_FillCircle(hjoy->stickPos.x, hjoy->stickPos.y, hjoy->stickPos.r);
      hjoy->stickPos.x = hjoy->pos.x;
      hjoy->stickPos.y = hjoy->pos.y;
    }
  }
  hjoy->xAxisPos = ((float)hjoy->stickPos.x - (float)hjoy->pos.x) / ((float)hjoy->pos.r - (float)hjoy->stickPos.r);
  hjoy->yAxisPos = ((float)hjoy->stickPos.y - (float)hjoy->pos.y) / ((float)hjoy->pos.r - (float)hjoy->stickPos.r);
  *hjoy->outputValX = hjoy->xDirectionCorrection * (hjoy->xAxisPos * hjoy->outputValMagnitude + hjoy->outputValCenter);
  *hjoy->outputValY = hjoy->yDirectionCorrection * (hjoy->yAxisPos * hjoy->outputValMagnitude + hjoy->outputValCenter);
  LCD_SetLayer(1);
  LCD_SetColor(hjoy->stickColor);
  LCD_FillCircle(hjoy->stickPos.x, hjoy->stickPos.y, hjoy->stickPos.r);
}

void UI(void)
{
  if (!hUI.curPage->ifPageInitialized)
  {
    hUI.curPage->PageInit();
    hUI.curPage->ifPageInitialized = 1;
  }
  hUI.curPage->Page();
}

void UI_Page_LowerLimb_Exoskeleton(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonPageSystemID);
  ButtonUpdate(&hButtonHipMotorZeroing);
  ButtonUpdate(&hButtonKneeMotorZeroing);
  ButtonUpdate(&hButtonMotorEnable);
  ButtonUpdate(&hButtonMotorDisable);
  ButtonUpdate(&hButtonPageGravityCompensation);
  ButtonUpdate(&hButtonPageExoskeletonParameterPanel);
  ButtonUpdate(&hButtonPageExoskeletonMuscularTorqueMonitor);
  ButtonUpdate(&hButtonDataLogStart);
  ButtonUpdate(&hButtonDataLogEnd);
  ButtonUpdate(&hButtonMotorProfilingStart);
  ButtonUpdate(&hButtonMotorProfilingEnd);
  
  
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(350, 100, hAKMotorRightHip.realPositionOffseted.f, 5, 1);
  LCD_DisplayDecimals(350, 125, hAKMotorRightKnee.realPositionOffseted.f, 5, 1);
  if (hAKMotorRightHip.status == AK10_9_Online)
    LCD_DisplayString(150, 100, "Online");
  else
    LCD_DisplayString(150, 100, "Offline");
  if (hAKMotorRightKnee.status == AK10_9_Online)
    LCD_DisplayString(150, 125, "Online");
  else
    LCD_DisplayString(150, 125, "Offline");
  
  LCD_DisplayDecimals(70, 250, hSystemID.sysIDResults_J1.f, 5, 3);
  LCD_DisplayDecimals(70, 275, hSystemID.sysIDResults_X1.f, 5, 3);
  LCD_DisplayDecimals(70, 300, hSystemID.sysIDResults_J2.f, 5, 3);
  LCD_DisplayDecimals(70, 325, hSystemID.sysIDResults_X2.f, 5, 3);
  
  if (ifButtonPressed(&hButtonPageSystemID))
  {
    UI_Page_Change_To(&UIPage_LowerLimb_SystemID);
    EXOSKELETON_SystemID_Init();
  }
  else if(ifButtonPressed(&hButtonHipMotorZeroing))
    AK10_9_MITMode_Zeroing(&hAKMotorRightHip);
  else if(ifButtonPressed(&hButtonKneeMotorZeroing))
    AK10_9_MITMode_Zeroing(&hAKMotorRightKnee);
  else if (ifButtonPressed(&hButtonMotorEnable))
  {
    AK10_9_MITMode_EnableMotor(&hAKMotorRightHip);
    AK10_9_MITMode_EnableMotor(&hAKMotorRightKnee);
  }
  else if (ifButtonPressed(&hButtonMotorDisable))
  {
    AK10_9_MITMode_DisableMotor(&hAKMotorRightHip);
    AK10_9_MITMode_DisableMotor(&hAKMotorRightKnee);
  }
  else if (ifButtonPressed(&hButtonPageGravityCompensation))
    UI_Page_Change_To(&UIPage_LowerLimb_GravityCompensation);
  else if (ifButtonPressed(&hButtonPageExoskeletonParameterPanel))
    UI_Page_Change_To(&UIPage_LowerLimb_ParameterAdjusting);
  else if (ifButtonPressed(&hButtonPageExoskeletonMuscularTorqueMonitor))
    UI_Page_Change_To(&UIPage_LowerLimb_MuscularTorqueMonitor);
  else if (ifButtonPressed(&hButtonDataLogStart))
    USB_DataLogStart();
  else if (ifButtonPressed(&hButtonDataLogEnd))
    USB_DataLogEnd();
  else if (ifButtonPressed(&hButtonMotorProfilingStart))
  {
    hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_MOTOR_PROFILING;
    hExoskeleton.hmotorprofiling->ifMotorProfiling = 1;
    hExoskeleton.hmotorprofiling->motorProfilingStartingTimestamp = HAL_GetTick();
  }
  else if (ifButtonPressed(&hButtonMotorProfilingEnd))
  {
    hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_FREE;
    hExoskeleton.hmotorprofiling->ifMotorProfiling = 0;
  }
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_LowerLimb_Exoskeleton_Init(void)
{
  hUI.task = UI_MAIN_TASK_LOWER_LIMB_EXOSKELETON;
  hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_FREE;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonPageSystemID = Button_Create(100, 50, 280, 40, "System Identification", LCD_WHITE, LCD_RED);
  hButtonHipMotorZeroing = Button_Create(100, 150, 280, 40, "Hip Joint Zeroing", LCD_WHITE, LCD_RED);
  hButtonKneeMotorZeroing = Button_Create(100, 200, 280, 40, "Knee Joint Zeroing", LCD_WHITE, LCD_RED);
  hButtonMotorEnable = Button_Create(170, 350, 280, 60, "Motor Enable", LIGHT_YELLOW, LCD_RED);
  hButtonMotorDisable = Button_Create(170, 420, 280, 100, "Motor Disable", LIGHT_YELLOW, LCD_RED);
  hButtonPageGravityCompensation = Button_Create(100, 530, 280, 60, "Gravity Compensation", LIGHT_GREEN, LCD_RED);
  hButtonPageExoskeletonParameterPanel = Button_Create(150, 250, 280, 60, "Parameter Adjusting", LIGHT_GREEN, LCD_RED);
  hButtonPageExoskeletonMuscularTorqueMonitor = Button_Create(100, 600, 280, 60, "Muscular Torque", LIGHT_GREEN, LCD_RED);
  hButtonDataLogStart = Button_Create(10, 350, 150, 60, "DatalogStart", LIGHT_YELLOW, LCD_RED);
  hButtonDataLogEnd = Button_Create(10, 420, 150, 60, "DatalogEnd", LIGHT_YELLOW, LCD_RED);
  hButtonMotorProfilingStart = Button_Create(10, 680, 150, 60, "MotrProfgSt", LIGHT_YELLOW, LCD_RED);
  hButtonMotorProfilingEnd = Button_Create(200, 680, 150, 60, "MotrProfgSp", LIGHT_YELLOW, LCD_RED);
  
  LCD_DisplayString(0, 100, "Hip  Joint:");
  LCD_DisplayString(250, 100, "Angle:");
  LCD_DisplayString(0, 125, "Knee Joint:");
  LCD_DisplayString(250, 125, "Angle:");
  
  LCD_DisplayString(0, 250, "J1 = ");
  LCD_DisplayString(0, 275, "X1 = ");
  LCD_DisplayString(0, 300, "J2 = ");
  LCD_DisplayString(0, 325, "X2 = ");
}

void UI_Page_LowerLimb_Exoskeleton_SystemID(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonStart);
  ButtonUpdate(&hButtonSystemIDJointMovementStart);
  ButtonUpdate(&hButtonStop);
  ButtonUpdate(&hButtonProfilingTimeIncrease);
  ButtonUpdate(&hButtonProfilingTimeDecrease);
  
  
  PotentialmeterUpdate(&hPotKneeProfilingFreq);
  PotentialmeterUpdate(&hPotKneeProfilingAmp);
  PotentialmeterUpdate(&hPotHipProfilingFreq);
  PotentialmeterUpdate(&hPotHipProfilingAmp);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_FREE)
    LCD_DisplayString(0, 70, "Pls start            ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_START)
    LCD_DisplayString(0, 70, "Starting...          ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
    LCD_DisplayString(0, 70, "Wait for knee task   ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_POSITIONING)
    LCD_DisplayString(0, 70, "Joints positioning...");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING)
    LCD_DisplayString(0, 70, "Knee learning...     ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
    LCD_DisplayString(0, 70, "Wait for hip task    ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_POSITIONING)
    LCD_DisplayString(0, 70, "Joints positioning...");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING)
    LCD_DisplayString(0, 70, "Hip learning...      ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS)
    LCD_DisplayString(0, 70, "Releasing...         ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS)
    LCD_DisplayString(0, 70, "Wait for results     ");
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_END)
    LCD_DisplayString(0, 70, "Ending...            ");
  
  LCD_DisplayDecimals(350, 700, hAKMotorRightHip.realPositionOffseted.f, 5, 1);
  LCD_DisplayDecimals(350, 725, hAKMotorRightKnee.realPositionOffseted.f, 5, 1);
  if (hAKMotorRightHip.status == AK10_9_Online)
    LCD_DisplayString(150, 700, "Online");
  else
    LCD_DisplayString(150, 700, "Offline");
  if (hAKMotorRightKnee.status == AK10_9_Online)
    LCD_DisplayString(150, 725, "Online");
  else
    LCD_DisplayString(150, 725, "Offline");
  
  if(hSystemID.ifIdentified)
    LCD_DisplayString(150, 200, "Identified    ");
  else
    LCD_DisplayString(150, 200, "Not identified");
  
  LCD_DisplayString(50, 250, "F_K:");
  LCD_DisplayDecimals(50, 275, hSystemID.kneeProfilingFreq, 2, 1);
  LCD_DisplayString(150, 250, "A_K:");
  LCD_DisplayDecimals(150, 275, hSystemID.kneeProfilingAmp, 2, 1);
  LCD_DisplayString(250, 250, "F_H:");
  LCD_DisplayDecimals(250, 275, hSystemID.hipProfilingFreq, 2, 1);
  LCD_DisplayString(350, 250, "A_H:");
  LCD_DisplayDecimals(350, 275, hSystemID.hipProfilingAmp, 2, 1);
  
  if (hUSB.datalogTask == DATALOG_TASK_DATALOG)
    LCD_DisplayString(200, 0, "Datalog   ready");
  else
    LCD_DisplayString(200, 0, "Datalog unready");
  
  if (ifButtonPressed(&hButtonStart))
  {
    AK10_9_MITMode_EnableMotor(&hAKMotorRightKnee);
    AK10_9_MITMode_EnableMotor(&hAKMotorRightHip);
    hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_START;
  }
  if (ifButtonPressed(&hButtonSystemIDJointMovementStart))
  {
    if (hUSB.datalogTask == DATALOG_TASK_DATALOG)
    {
      if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_POSITIONING;
      else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_POSITIONING;
    }
  }
  if (ifButtonPressed(&hButtonStop))
  {
    USB_SendText("STOP");
    AK10_9_MITMode_DisableMotor(&hAKMotorRightKnee);
    AK10_9_MITMode_DisableMotor(&hAKMotorRightHip);
    hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_FREE;
    hUSB.datalogTask = DATALOG_TASK_FREE;
  }
  if (ifButtonPressed(&hButtonProfilingTimeIncrease))
  {
    if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.kneeProfilingTime += 5000;
    else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.hipProfilingTime += 5000;
    LIMIT_MIN_MAX(hSystemID.kneeProfilingTime, 0, 600000);
    LIMIT_MIN_MAX(hSystemID.hipProfilingTime, 0, 600000);
  }
  else if (ifButtonPressed(&hButtonProfilingTimeDecrease))
  {
    if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.kneeProfilingTime -= 5000;
    else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.hipProfilingTime -= 5000;
    LIMIT_MIN_MAX(hSystemID.kneeProfilingTime, 0, 600000);
    LIMIT_MIN_MAX(hSystemID.hipProfilingTime, 0, 600000);
  }
  
  if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
    LCD_DisplayDecimals(350, 570, (float)hSystemID.kneeProfilingTime/1000.0f, 3, 0);
  else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
    LCD_DisplayDecimals(350, 570, (float)hSystemID.hipProfilingTime/1000.0f, 3, 0);
  else
    LCD_DisplayString(350, 570, "NA          ");
    
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
}
void UI_Page_LowerLimb_Exoskeleton_SystemID_Init(void)
{
  hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_SYSTEM_ID;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonStart = Button_Create(0, 100, 110, 40, "Start", LCD_WHITE, LCD_RED);
  hButtonSystemIDJointMovementStart = Button_Create(150, 100, 300, 40, "Start Joint Motion", LCD_WHITE, LCD_RED);
  hButtonStop = Button_Create(0, 150, 110, 80, "Stop", LCD_YELLOW, LCD_RED);
  hButtonProfilingTimeIncrease = Button_Create(50, 570, 250, 40, "Duration +10s", LCD_WHITE, LCD_RED);
  hButtonProfilingTimeDecrease = Button_Create(50, 620, 250, 40, "Duration -10s", LCD_WHITE, LCD_RED);
  
  hPotKneeProfilingFreq = Potentialmeter_Create(50, 250, 30, 300, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 1.0f, 0.0f, &hSystemID.kneeProfilingFreq);
  hPotKneeProfilingAmp = Potentialmeter_Create(150, 250, 30, 300, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 45.0f, 0.0f, &hSystemID.kneeProfilingAmp);
  hPotHipProfilingFreq = Potentialmeter_Create(250, 250, 30, 300, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 1.0f, 0.0f, &hSystemID.hipProfilingFreq);
  hPotHipProfilingAmp = Potentialmeter_Create(350, 250, 30, 300, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 90.0f, 0.0f, &hSystemID.hipProfilingAmp);
  
  LCD_DisplayString(0, 700, "Hip  Joint:");
  LCD_DisplayString(250, 700, "Angle:");
  LCD_DisplayString(0, 725, "Knee Joint:");
  LCD_DisplayString(250, 725, "Angle:");
}

void UI_Page_LowerLimb_Exoskeleton_GravityCompensation(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonStart);
  ButtonUpdate(&hButtonStop);
  PotentialmeterUpdate(&hPotGravityCompensationHipThrottle);
  PotentialmeterUpdate(&hPotGravityCompensationKneeThrottle);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(280, 215, hGravityCompensation.throttleKnee, 4, 3);
  LCD_DisplayDecimals(380, 215, hGravityCompensation.throttleHip, 4, 3);
  LCD_DisplayDecimals(350, 0, hAKMotorRightHip.realTorque.f, 4, 1);
  LCD_DisplayDecimals(350, 30, hAKMotorRightKnee.realTorque.f, 4, 1);
  LCD_DisplayDecimals(350, 60, hGravityCompensation.torqueDesiredHip.f, 4, 1);
  LCD_DisplayDecimals(350, 90, hGravityCompensation.torqueDesiredKnee.f, 4, 1);
  
  if (ifButtonPressed(&hButtonStart))
  {
    hGravityCompensation.ifGravityCompensationStarted = 1;
  }
  else if (ifButtonPressed(&hButtonStop))
  {
    hGravityCompensation.ifGravityCompensationStarted = 0;
    AK10_9_MITMode_DisableMotor(&hAKMotorRightKnee);
    AK10_9_MITMode_DisableMotor(&hAKMotorRightHip);
  }
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
}
void UI_Page_LowerLimb_Exoskeleton_GravityCompensation_Init(void)
{
  hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_GRAVITY_COMPENSATION;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonStart = Button_Create(0, 100, 100, 150, "Start", LCD_BLUE, LCD_RED);
  hButtonStop = Button_Create(0, 300, 200, 250, "Stop", LCD_YELLOW, LCD_RED);
  LCD_DisplayString(280, 190, "Knee%:");
  LCD_DisplayString(380, 190, "Hip %:");
  LCD_DisplayString(80, 0, "Hip Torque rel/Nm:  ");
  LCD_DisplayString(80, 30,"Knee Torque rel/Nm: ");
  LCD_DisplayString(130, 60, "Hip Torque des/Nm:  ");
  LCD_DisplayString(130, 90,"Knee Torque des/Nm: ");
  
  hPotGravityCompensationHipThrottle = Potentialmeter_Create(400, 250, 30, 400, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 1.0f, 0.0f, &hGravityCompensation.throttleHip);
  hPotGravityCompensationKneeThrottle = Potentialmeter_Create(300, 250, 30, 400, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 1.0f, 0.0f, &hGravityCompensation.throttleKnee);
  EXOSKELETON_GravityCompensation_Init(&hGravityCompensation);
}

void UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting(void)
{
  ButtonUpdate(&hButtonGoBack);
  PotentialmeterUpdate(&hPotParameterAdjust_L1);
  PotentialmeterUpdate(&hPotParameterAdjust_J1);
  PotentialmeterUpdate(&hPotParameterAdjust_X1);
  PotentialmeterUpdate(&hPotParameterAdjust_J2);
  PotentialmeterUpdate(&hPotParameterAdjust_X2);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(0, 75, hExoskeleton.L1.f, 5, 4);
  LCD_DisplayDecimals(80, 75, hExoskeleton.hsysid->sysIDResults_J1.f, 5, 4);
  LCD_DisplayDecimals(160, 75, hExoskeleton.hsysid->sysIDResults_X1.f, 5, 4);
  LCD_DisplayDecimals(240, 75, hExoskeleton.hsysid->sysIDResults_J2.f, 5, 4);
  LCD_DisplayDecimals(320, 75, hExoskeleton.hsysid->sysIDResults_X2.f, 5, 4);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
}
void UI_Page_LowerLimb_Exoskeleton_ParameterAdjusting_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  LCD_DisplayString(30, 100, "L1");
  LCD_DisplayString(110, 100, "J1");
  LCD_DisplayString(190, 100, "X1");
  LCD_DisplayString(270, 100, "J2");
  LCD_DisplayString(350, 100, "X2");
  hPotParameterAdjust_L1 = Potentialmeter_Create(30, 100, 30, 630, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 0.7f, 0.0f, &hExoskeleton.L1.f);
  PotentialmeterSliderGoTo(&hPotParameterAdjust_L1, hExoskeleton.L1.f);
  
  hPotParameterAdjust_J1 = Potentialmeter_Create(110, 100, 30, 630, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 2.0f, 0.0f, &hExoskeleton.hsysid->sysIDResults_J1.f);
  PotentialmeterSliderGoTo(&hPotParameterAdjust_J1, hExoskeleton.hsysid->sysIDResults_J1.f);
  
  hPotParameterAdjust_X1 = Potentialmeter_Create(190, 100, 30, 630, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 6.0f, 0.0f, &hExoskeleton.hsysid->sysIDResults_X1.f);
  PotentialmeterSliderGoTo(&hPotParameterAdjust_X1, hExoskeleton.hsysid->sysIDResults_X1.f);
  
  hPotParameterAdjust_J2 = Potentialmeter_Create(270, 100, 30, 630, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 0.4f, 0.0f, &hExoskeleton.hsysid->sysIDResults_J2.f);
  PotentialmeterSliderGoTo(&hPotParameterAdjust_J2, hExoskeleton.hsysid->sysIDResults_J2.f);
  
  hPotParameterAdjust_X2 = Potentialmeter_Create(350, 100, 30, 630, 130, 70, \
                          LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 2.0f, 0.0f, &hExoskeleton.hsysid->sysIDResults_X2.f);
  PotentialmeterSliderGoTo(&hPotParameterAdjust_X2, hExoskeleton.hsysid->sysIDResults_X2.f);
}

void UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonOn);
  ButtonUpdate(&hButtonOff);
  ButtonUpdate(&hButtonDataLogStart);
  ButtonUpdate(&hButtonDataLogEnd);
  ButtonUpdate(&hButtonAugmentedControlOff);;
  ButtonUpdate(&hButtonAugmentedControlOn);
  PotentialmeterUpdate(&hPotAugmentedControlHipThrottle);
  PotentialmeterUpdate(&hPotAugmentedControlKneeThrottle);
  
  
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(300, 0, hExoskeleton.hmusculartorque->muscularTorqueHip.f, 5, 1);
  LCD_DisplayDecimals(300, 25, hExoskeleton.hmusculartorque->muscularTorqueKnee.f, 5, 1);
  LCD_DisplayDecimals(340, 75, hExoskeleton.haugmentedcontrol->hipJointAugmentedControlThrottle, 2,1);
  LCD_DisplayDecimals(390, 75, hExoskeleton.haugmentedcontrol->kneeJointAugmentedControlThrottle, 2,1);
  
  
  if (ifButtonPressed(&hButtonOn))
    hExoskeleton.hmusculartorque->ifEstimating = 1;
  else if (ifButtonPressed(&hButtonOff))
    hExoskeleton.hmusculartorque->ifEstimating = 0;
  else if (ifButtonPressed(&hButtonDataLogStart))
    USB_DataLogStart();
  else if (ifButtonPressed(&hButtonDataLogEnd))
    USB_DataLogEnd();
  else if (ifButtonPressed(&hButtonAugmentedControlOn))
  {
    hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_AUGMENTATION_CONTROL;
    hExoskeleton.haugmentedcontrol->ifAugmentedControl = 1;
  }
  else if (ifButtonPressed(&hButtonAugmentedControlOff))
  {
    hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_FREE;
    hExoskeleton.haugmentedcontrol->ifAugmentedControl = 0;
  }
  
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
}
void UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor_Init(void)
{
  USB_SetNewDataSlotLen(sizeof(dataSlots_Exoskeleton_Common)/4);
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonOn = Button_Create(100, 100, 100, 70, "On", LCD_WHITE, LCD_RED);
  hButtonOff = Button_Create(100, 200, 100, 70, "Off", LCD_WHITE, LCD_RED);
  hButtonDataLogStart = Button_Create(100, 300, 200, 70, "Datalog Start", LCD_WHITE, LCD_RED);
  hButtonDataLogEnd = Button_Create(100, 400, 200, 70, "Datalog End", LCD_WHITE, LCD_RED);
  hButtonAugmentedControlOn = Button_Create(100, 500, 200, 70, "Augmented Control On", LCD_WHITE, LCD_RED);
  hButtonAugmentedControlOff = Button_Create(100, 600, 200, 70, "Augmented Control Off", LCD_WHITE, LCD_RED);
  hPotAugmentedControlHipThrottle = Potentialmeter_Create(350, 100, 20, 400, 130, 40, \
                                    LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 2.0f, 0.0f, &hExoskeleton.haugmentedcontrol->hipJointAugmentedControlThrottle);
  hPotAugmentedControlKneeThrottle = Potentialmeter_Create(400, 100, 20, 400, 130, 40, \
                                    LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 2.0f, 0.0f, &hExoskeleton.haugmentedcontrol->kneeJointAugmentedControlThrottle);
  
  LCD_DisplayString(200, 0, "Hip/Nm:  ");
  LCD_DisplayString(200, 25,"Knee/Nm: ");
  LCD_DisplayString(340, 100, "hip");
  LCD_DisplayString(390, 100, "knee");
}

void UI_Page_Home1(void)
{
  LCD_SetFont(&Font32); 
  LCD_DisplayString(140, 30, "Welcome Jiaye");
  LCD_SetFont(&Font24);
  ButtonUpdate(&hButtonPageExoskeletonInterface);
  ButtonUpdate(&hButtonPageAK10_9ManualControl);
  ButtonUpdate(&hButtonPageBNO055_Monitor);
  ButtonUpdate(&hButtonPageAccelerationObserverProject);
  ButtonUpdate(&hButtonPageADCMonitor);
  ButtonUpdate(&hButtonPageCustomizedIMUMonitor);
  ButtonUpdate(&hButtonPageLinKongKeJiTesting);
  ButtonUpdate(&hButtonPageTorqueConstantCalibration);
  
  
  if (ifButtonPressed(&hButtonPageExoskeletonInterface))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
  if (ifButtonPressed(&hButtonPageAK10_9ManualControl))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControlCubeMarsFWMITMode);
  if (ifButtonPressed(&hButtonPageBNO055_Monitor))
    UI_Page_Change_To(&UIPage_BNO055_Monitor);
  if (ifButtonPressed(&hButtonPageAccelerationObserverProject))
    UI_Page_Change_To(&UIPage_Acceleration_Observer_Project);
  if (ifButtonPressed(&hButtonPageADCMonitor))
    UI_Page_Change_To(&UIPage_ADC_Monitor);
  if (ifButtonPressed(&hButtonPageCustomizedIMUMonitor))
    UI_Page_Change_To(&UIPage_Customized_IMU_Monitor);
  if (ifButtonPressed(&hButtonPageLinKongKeJiTesting))
    UI_Page_Change_To(&UIPage_LinKongKeJi_Testing);
  if (ifButtonPressed(&hButtonPageTorqueConstantCalibration))
    UI_Page_Change_To(&UIPage_TorqueConstantCalibration);
}
void UI_Page_Home1_Init(void)
{
  hUI.task = UI_MAIN_TASK_NONE;
  hButtonPageExoskeletonInterface = Button_Create(100, 100, 300, 40, "Lower Limb Exoskeleton", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAK10_9ManualControl = Button_Create(80, 150, 360, 40, "AK10-9 V2.0 Manual Control", LIGHT_MAGENTA, LCD_RED);
  hButtonPageBNO055_Monitor = Button_Create(150, 250, 200, 40, "BNO055 Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAccelerationObserverProject = Button_Create(10, 300, 450, 40, "Acceleration Observer Project", LIGHT_MAGENTA, LCD_RED);
  hButtonPageADCMonitor = Button_Create(150, 350, 200, 40, "ADC Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageCustomizedIMUMonitor = Button_Create(80, 200, 360, 40, "Customized IMU Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageLinKongKeJiTesting = Button_Create(100, 450, 300, 40, "LinKongKeJi MG Motor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageTorqueConstantCalibration = Button_Create(100, 550, 300, 40, "Kt Calibration", LIGHT_MAGENTA, LCD_RED);

}

void UI_Page_AK10_9_ManualControlCubeMarsFWServoMode(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonMotorStart);
  ButtonUpdate(&hButtonMotorStop);
  ButtonUpdate(&hButtonMotorZeroing);
  ButtonUpdate(&hButtonManualControlMode);
  ButtonUpdate(&hButtonMotorSelectRightHip);
  ButtonUpdate(&hButtonMotorSelectRightKnee);
  PotentialmeterUpdate(&hTMotorManualControlPot_pos);
  PotentialmeterUpdate(&hTMotorManualControlPot_vel);
  PotentialmeterUpdate(&hTMotorManualControlPot_cur);
  
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_ServoMode_Zeroing(hMotorPtrManualControl);
  if(ifButtonPressed(&hButtonMotorStart))
    ifManualControlStarted = 1;
  if(ifButtonPressed(&hButtonMotorStop))
  {
    ifManualControlStarted = 0;
    LCD_ClearRect(0, 370, 400, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
  }
  if(ifButtonPressed(&hButtonManualControlMode))
  {
    LCD_ClearRect(0, 370, 200, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    ifManualControlStarted = 0;
    controlModeCubeMarsFW++;
    if ((uint32_t)controlModeCubeMarsFW > 3)
      controlModeCubeMarsFW = 0;
  }
  if (ifButtonPressed(&hButtonMotorSelectRightHip))
  {
    ifManualControlStarted = 0;
    hMotorPtrManualControl = &hAKMotorRightHip;
    LCD_ClearRect(0, 370, 400, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    hButtonMotorSelectRightHip.colorUnpressed = LIGHT_YELLOW;
    hButtonMotorSelectRightKnee.colorUnpressed = DARK_YELLOW;
    hButtonMotorSelectRightHip.ifNeedRefresh = 1;
    hButtonMotorSelectRightKnee.ifNeedRefresh = 1;
  }
  else if (ifButtonPressed(&hButtonMotorSelectRightKnee))
  {
    ifManualControlStarted = 0;
    hMotorPtrManualControl = &hAKMotorRightKnee;
    LCD_ClearRect(0, 370, 400, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    hButtonMotorSelectRightHip.colorUnpressed = DARK_YELLOW;
    hButtonMotorSelectRightKnee.colorUnpressed = LIGHT_YELLOW;
    hButtonMotorSelectRightHip.ifNeedRefresh = 1;
    hButtonMotorSelectRightKnee.ifNeedRefresh = 1;
  }
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_CURRENT)
  {
    LCD_DisplayString(0, 340, "Current  Control");
    LCD_SetColor(LCD_RED);
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_cur, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_POSITION)
  {
    LCD_DisplayString(0, 340, "Position Control");
    LCD_SetColor(LCD_RED);
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_pos, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_VELOCITY)
  {
    LCD_DisplayString(0, 340, "Velocity Control");
    LCD_SetColor(LCD_RED);
    
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_vel, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlModeCubeMarsFW == AK10_9_CUBEMARS_FW_MODE_BRAKE)
    LCD_DisplayString(0, 340, "     BRAKE      ");
  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  LCD_DisplayDecimals(400, 735, (double)hMotorPtrManualControl->realTorque.f, 3, 2);
  LCD_DisplayNumber(400, 760, hMotorPtrManualControl->temperature, 2);
  LCD_DisplayDecimals(150, 710, (double)hMotorPtrManualControl->realPosition.f, 6, 3);
  LCD_DisplayDecimals(150, 735, (double)hMotorPtrManualControl->realVelocityPresent.f, 6, 3);
  LCD_DisplayDecimals(150, 760, (double)hMotorPtrManualControl->realCurrent.f, 6, 3);
  LCD_DisplayDecimals(250, 710, (double)hMotorPtrManualControl->setPos.f, 6, 3);
  LCD_DisplayDecimals(250, 735, (double)hMotorPtrManualControl->setVel.f, 6, 3);
  LCD_DisplayDecimals(250, 760, (double)hMotorPtrManualControl->setIq.f, 6, 3);
  
  if (ifButtonPressed(&hButtonGoBack))
  {
    UI_Page_Change_To(&UIPage_Home1);
    ifManualControlStarted = 0;
  }
}
void UI_Page_AK10_9_ManualControlCubeMarsFWServoMode_Init(void)
{
  hUI.task = UI_MAIN_TASK_AK10_9_MANUAL_CONTROL;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorStart = Button_Create(0, 420, 100, 40, "START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(0, 80, 150, 250, "STOP", LCD_RED, LCD_YELLOW);
  hButtonMotorZeroing = Button_Create(0, 620, 200, 40, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonManualControlMode = Button_Create(0, 470, 160, 40, "Control Mode", LCD_WHITE, LCD_RED);
  hButtonMotorSelectRightHip = Button_Create(0, 520, 160, 40, "Right Hip", DARK_YELLOW, LCD_RED);
  hButtonMotorSelectRightKnee = Button_Create(0, 570, 160, 40, "Right Knee", DARK_YELLOW, LCD_RED);
  hTMotorManualControlPot_pos = Potentialmeter_Create(250, 80, 30, 550, 130, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -500.0f, 500.0f, 0.0f, &manualControlValue_pos);
  hTMotorManualControlPot_vel = Potentialmeter_Create(340, 80, 30, 550, 130, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -500.0f, 500.0f, 0.0f, &manualControlValue_vel);
  hTMotorManualControlPot_cur = Potentialmeter_Create(420, 80, 30, 550, 130, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -50.0f, 50.0f, 0.0f, &manualControlValue_cur);
  
  LCD_DisplayString(250, 50, "pos");
  LCD_DisplayString(340, 50, "vel");
  LCD_DisplayString(420, 50, "cur");
  LCD_DisplayString(170, 685, "mes");
  LCD_DisplayString(270, 685, "des");
  LCD_DisplayString(10, 710, "Position: ");
  LCD_DisplayString(10, 735, "Velocity: ");
  LCD_DisplayString(10, 760, "Current:  ");
  LCD_DisplayString(330, 760, "Temp:");
  LCD_DisplayString(330, 735, "T(Nm):");
  
  hMotorPtrManualControl = NULL;
}

void UI_Page_AK10_9_ManualControlCubeMarsFWMITMode(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonMotorStart);
  ButtonUpdate(&hButtonMotorStop);
  ButtonUpdate(&hButtonMotorZeroing);
  ButtonUpdate(&hButtonMotorSelectRightHip);
  ButtonUpdate(&hButtonMotorSelectRightKnee);
  PotentialmeterUpdate(&hTMotorManualControlPot_pos);
  PotentialmeterUpdate(&hTMotorManualControlPot_vel);
  PotentialmeterUpdate(&hTMotorManualControlPot_cur);
  PotentialmeterUpdate(&hTMotorManualControlPot_kp);
  PotentialmeterUpdate(&hTMotorManualControlPot_kd);
  
  if (ifButtonPressed(&hButtonMotorSelectRightHip))
  {
    ifManualControlStarted = 0;
    hMotorPtrManualControl = &hAKMotorRightHip;
    LCD_ClearRect(0, 370, 400, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    hButtonMotorSelectRightHip.colorUnpressed = LIGHT_YELLOW;
    hButtonMotorSelectRightKnee.colorUnpressed = DARK_YELLOW;
    hButtonMotorSelectRightHip.ifNeedRefresh = 1;
    hButtonMotorSelectRightKnee.ifNeedRefresh = 1;
  }
  else if (ifButtonPressed(&hButtonMotorSelectRightKnee))
  {
    ifManualControlStarted = 0;
    hMotorPtrManualControl = &hAKMotorRightKnee;
    LCD_ClearRect(0, 370, 400, 30);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    hButtonMotorSelectRightHip.colorUnpressed = DARK_YELLOW;
    hButtonMotorSelectRightKnee.colorUnpressed = LIGHT_YELLOW;
    hButtonMotorSelectRightHip.ifNeedRefresh = 1;
    hButtonMotorSelectRightKnee.ifNeedRefresh = 1;
  }
  
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_MITMode_Zeroing(hMotorPtrManualControl);
  if(ifButtonPressed(&hButtonMotorStart))
  {
    AK10_9_MITMode_EnableMotor(hMotorPtrManualControl);
    ifManualControlStarted = 1;
    
  }
  if(ifButtonPressed(&hButtonMotorStop))
  {
    AK10_9_MITMode_DisableMotor(hMotorPtrManualControl);
    ifManualControlStarted = 0;
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
  }
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);

  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  LCD_DisplayDecimals(400, 735, (double)hMotorPtrManualControl->realTorque.f, 3, 2);
  LCD_DisplayDecimals(150, 710, (double)hMotorPtrManualControl->realPosition.f, 6, 3);
  LCD_DisplayDecimals(150, 735, (double)hMotorPtrManualControl->realVelocityPresent.f, 6, 3);
  LCD_DisplayDecimals(150, 760, (double)hMotorPtrManualControl->realCurrent.f, 6, 3);
  LCD_DisplayDecimals(250, 710, (double)hMotorPtrManualControl->setPos.f, 6, 3);
  LCD_DisplayDecimals(250, 735, (double)hMotorPtrManualControl->setVel.f, 6, 3);
  LCD_DisplayDecimals(250, 760, (double)hMotorPtrManualControl->setIq.f, 6, 3);
  LCD_DisplayDecimals(250, 80, (double)manualControlValue_pos, 3, 1);
  LCD_DisplayDecimals(340, 80, (double)manualControlValue_vel, 3, 1);
  LCD_DisplayDecimals(420, 80, (double)manualControlValue_cur, 3, 1);
  LCD_DisplayDecimals(340, 510, (double)manualControlValue_kp, 3, 0);
  LCD_DisplayDecimals(420, 510, (double)manualControlValue_kd, 3, 2);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_AK10_9_ManualControlCubeMarsFWMITMode_Init(void)
{
  hUI.task = UI_MAIN_TASK_AK10_9_MANUAL_CONTROL;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorStart = Button_Create(0, 420, 100, 40, "START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(0, 80, 150, 250, "STOP", LCD_RED, LCD_YELLOW);
  hButtonMotorZeroing = Button_Create(0, 620, 200, 40, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonMotorSelectRightHip = Button_Create(0, 520, 160, 40, "Right Hip", DARK_YELLOW, LCD_RED);
  hButtonMotorSelectRightKnee = Button_Create(0, 570, 160, 40, "Right Knee", DARK_YELLOW, LCD_RED);
  hTMotorManualControlPot_pos = Potentialmeter_Create(250, 80, 30, 400, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 180.0f, 0.0f, &manualControlValue_pos);
  hTMotorManualControlPot_vel = Potentialmeter_Create(340, 80, 30, 400, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -800.0f, 800.0f, 0.0f, &manualControlValue_vel);
  hTMotorManualControlPot_cur = Potentialmeter_Create(420, 80, 30, 400, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -18.0f, 18.0f, 0.0f, &manualControlValue_cur);
  hTMotorManualControlPot_kp = Potentialmeter_Create(340, 510, 30, 200, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 500.0f, 0.0f, &manualControlValue_kp);
  hTMotorManualControlPot_kd = Potentialmeter_Create(420, 510, 30, 200, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 5.0f, 0.0f, &manualControlValue_kd);
  
  
  LCD_DisplayString(250, 50, "pos");
  LCD_DisplayString(340, 50, "vel");
  LCD_DisplayString(420, 50, "cur");
  LCD_DisplayString(340, 480, "kp");
  LCD_DisplayString(420, 480, "kd");
  LCD_DisplayString(170, 685, "mes");
  LCD_DisplayString(270, 685, "des");
  LCD_DisplayString(10, 710, "Position: ");
  LCD_DisplayString(10, 735, "Velocity: ");
  LCD_DisplayString(10, 760, "Current:  ");
  LCD_DisplayString(330, 735, "T(Nm):");
}

void UI_Page_BNO055_Monitor(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonIMUSetModeNDOF);
  ButtonUpdate(&hButtonIMUSetModeACCONLY);
  ButtonUpdate(&hButtonIMUSetModeGYROONLY);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(200, 0, (double)hIMUTorso.rawData.liaccX.b16, 7, 1);
  LCD_DisplayDecimals(200, 50, (double)hIMUTorso.rawData.liaccY.b16, 7, 1);
  LCD_DisplayDecimals(200, 100, (double)hIMUTorso.rawData.liaccZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 150, (double)hIMUTorso.rawData.AccX.b16, 7, 1);
  LCD_DisplayDecimals(200, 200, (double)hIMUTorso.rawData.AccY.b16, 7, 1);
  LCD_DisplayDecimals(200, 250, (double)hIMUTorso.rawData.AccZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 300, (double)hIMUTorso.rawData.gyroX.b16, 7, 1);
  LCD_DisplayDecimals(200, 350, (double)hIMUTorso.rawData.gyroY.b16, 7, 1);
  LCD_DisplayDecimals(200, 400, (double)hIMUTorso.rawData.gyroZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 450, (double)hIMUTorso.rawData.MagX.b16, 7, 1);
  LCD_DisplayDecimals(200, 500, (double)hIMUTorso.rawData.MagY.b16, 7, 1);
  LCD_DisplayDecimals(200, 550, (double)hIMUTorso.rawData.MagZ.b16, 7, 1);
  
  if (hIMUTorso.operationModeENUM == IMU_MODE_NDOF)
    EXOSKELETON_SetBNO055Mode_9_DOF(&hIMUTorso);
  else if (hIMUTorso.operationModeENUM == IMU_MODE_ACCONLY)
    EXOSKELETON_SetBNO055Mode_ACC_Only(&hIMUTorso);
  else if (hIMUTorso.operationModeENUM == IMU_MODE_GYROONLY)
    EXOSKELETON_SetBNO055Mode_GYRO_Only(&hIMUTorso);
  
  if (ifButtonPressed(&hButtonIMUSetModeNDOF))
    hIMUTorso.operationModeENUM = IMU_MODE_NDOF;
  if (ifButtonPressed(&hButtonIMUSetModeACCONLY))
    hIMUTorso.operationModeENUM = IMU_MODE_ACCONLY;
  if (ifButtonPressed(&hButtonIMUSetModeGYROONLY))
    hIMUTorso.operationModeENUM = IMU_MODE_GYROONLY;
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_BNO055_Monitor_Init(void)
{
  hUI.task = UI_MAIN_TASK_BNO055_MONITOR;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonIMUSetModeNDOF = Button_Create(100, 700, 250, 40, "9 DOF Fusion Mode", LIGHT_YELLOW, LCD_RED);
  hButtonIMUSetModeACCONLY = Button_Create(100, 750, 200, 40, "ACCONLY Mode", LIGHT_YELLOW, LCD_RED);
  hButtonIMUSetModeGYROONLY = Button_Create(100, 650, 200, 40, "GYROONLY Mode", LIGHT_YELLOW, LCD_RED);
  LCD_DisplayString(100, 0, "LiAccX: ");
  LCD_DisplayString(100, 50, "LiAccY: ");
  LCD_DisplayString(100, 100, "LiAccZ: ");
  LCD_DisplayString(100, 150, "AccX: ");
  LCD_DisplayString(100, 200, "AccY: ");
  LCD_DisplayString(100, 250, "AccZ: ");
  LCD_DisplayString(100, 300, "GyroX:  ");
  LCD_DisplayString(100, 350, "GyroY:  ");
  LCD_DisplayString(100, 400, "GyroZ:  ");
  LCD_DisplayString(100, 450, "MagX:  ");
  LCD_DisplayString(100, 500, "MagY:  ");
  LCD_DisplayString(100, 550, "MagZ:  ");
}

void UI_Page_Change_To(PageHandle* hpage)
{
  hUI.prePage = hUI.curPage;
  hUI.curPage = hpage;
  hUI.curPage->ifPageInitialized = 0;
  
  LCD_SetLayer(0); 
	LCD_SetBackColor(LCD_CYAN);
	LCD_SetColor(LCD_BLACK);
	LCD_Clear();
  LCD_SetLayer(1);
  LCD_Clear();
}

void UI_Page_Acceleration_Observer_Project(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonDataLogStart);
  ButtonUpdate(&hButtonDataLogEnd);
  ButtonUpdate(&hButtonMotorProfilingStart);
  ButtonUpdate(&hButtonMotorProfilingEnd);
  ButtonUpdate(&hButtonMotorZeroing);
  
  if (ifButtonPressed(&hButtonDataLogStart))
    USB_DataLogStart();
  if (ifButtonPressed(&hButtonDataLogEnd))
    USB_DataLogEnd();
  if(ifButtonPressed(&hButtonMotorProfilingStart))
  {
    ifMotorProfilingStarted = 1;
    timeDifference = HAL_GetTick();
  }
  if(ifButtonPressed(&hButtonMotorProfilingEnd))
  {
    ifMotorProfilingStarted = 0;
    timeDifference = 0;
  }
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_ServoMode_Zeroing(&hAKMotorRightHip);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hAKMotorRightHip.status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  
  LCD_DisplayDecimals(140, 720, (double)hAKMotorRightHip.realPosition.f, 10, 4);
  LCD_DisplayDecimals(140, 745, (double)hAKMotorRightHip.setPos.f, 10, 4);
  LCD_DisplayDecimals(140, 770, (double)hAKMotorRightHip.realVelocityPresent.f, 10, 4);
  LCD_SetColor(DARK_RED);
  LCD_DisplayDecimals(90, 495, (double)hIMUTorso.rawData.liaccX.b16, 7, 1);
  LCD_DisplayDecimals(90, 520, (double)hIMUTorso.rawData.liaccY.b16, 7, 1);
  LCD_DisplayDecimals(90, 545, (double)hIMUTorso.rawData.liaccZ.b16, 7, 1);
  LCD_DisplayDecimals(90, 570, (double)hIMUTorso.rawData.AccX.b16, 7, 1);
  LCD_DisplayDecimals(90, 595, (double)hIMUTorso.rawData.AccY.b16, 7, 1);
  LCD_DisplayDecimals(90, 620, (double)hIMUTorso.rawData.AccZ.b16, 7, 1);
  LCD_DisplayDecimals(90, 645, (double)hIMUTorso.rawData.gyroX.b16, 7, 1);
  LCD_DisplayDecimals(90, 670, (double)hIMUTorso.rawData.gyroY.b16, 7, 1);
  LCD_DisplayDecimals(90, 695, (double)hIMUTorso.rawData.gyroZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 470, (double)tmotorProfilingSinWaveFrequency, 3, 4);
  LCD_DisplayDecimals(230, 445, (double)(hAKMotorRightHip.realPosition.f - hAKMotorRightHip.setPos.f), 3, 4);
  LCD_SetColor(LCD_BLACK);
  
  ifIMUFeedbackStarted = 1;
  if (ifButtonPressed(&hButtonGoBack))
  {
    UI_Page_Change_To(&UIPage_Home1);
    ifIMUFeedbackStarted = 0;
  }
}
void UI_Page_Acceleration_Observer_Project_Init(void)
{
  hUI.task = UI_MAIN_TASK_TMOTOR_ACCELERATION_OBSERVER;
  hButtonDataLogStart = Button_Create(10, 100, 200, 40, "Data Log Start", LIGHT_MAGENTA, LCD_RED);
  hButtonDataLogEnd = Button_Create(10, 150, 200, 40, "Data Log End", LCD_GREEN, LCD_RED);
  hButtonMotorProfilingStart = Button_Create(10, 200, 300, 40, "Motor profiling Start", LIGHT_GREY, LCD_RED);
  hButtonMotorProfilingEnd = Button_Create(10, 250, 300, 40, "Motor profiling Stop", LCD_YELLOW, LCD_RED);
  hButtonMotorZeroing = Button_Create(10, 330, 200, 40, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
 
  LCD_SetLayer(0); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayString(10, 720, "Position mes: ");
  LCD_DisplayString(10, 745, "Position des: ");
  LCD_DisplayString(10, 770, "Velocity: ");
  LCD_SetColor(DARK_RED);
  LCD_DisplayString(10, 445, "Position error: ");
  LCD_DisplayString(10, 470, "Profile Freq: ");
  LCD_DisplayString(10, 495, "LiAccX: ");
  LCD_DisplayString(10, 520, "LiAccY: ");
  LCD_DisplayString(10, 545, "LiAccZ: ");
  LCD_DisplayString(10, 570, "AccX: ");
  LCD_DisplayString(10, 595, "AccY: ");
  LCD_DisplayString(10, 620, "AccZ: ");
  LCD_DisplayString(10, 645, "GyroX:  ");
  LCD_DisplayString(10, 670, "GyroY:  ");
  LCD_DisplayString(10, 695, "GyroZ:  ");
  
}

void UI_Page_ADC_Monitor_Init(void)
{
  hUI.task = UI_MAIN_TASK_ADC_MONITOR;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonResetAD7606 = Button_Create(100, 500, 300, 50, "Reset AD7606", LCD_GREEN, LCD_RED);
  LCD_DisplayString(100, 200, "Channel 1 (V): ");
  LCD_DisplayString(100, 225, "Channel 2 (V): ");
  LCD_DisplayString(100, 250, "Channel 3 (V): ");
  LCD_DisplayString(100, 275, "Channel 4 (V): ");
  LCD_DisplayString(100, 300, "Channel 5 (V): ");
  LCD_DisplayString(100, 325, "Channel 6 (V): ");
  LCD_DisplayString(100, 350, "Channel 7 (V): ");
  LCD_DisplayString(100, 375, "Channel 8 (V): ");
}
void UI_Page_ADC_Monitor(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonResetAD7606);
  
  LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(300, 200, hADC.volt[0], 5, 4);
  LCD_DisplayDecimals(300, 225, hADC.volt[1], 5, 4);
  LCD_DisplayDecimals(300, 250, hADC.volt[2], 5, 4);
  LCD_DisplayDecimals(300, 275, hADC.volt[3], 5, 4);
  LCD_DisplayDecimals(300, 300, hADC.volt[4], 5, 4);
  LCD_DisplayDecimals(300, 325, hADC.volt[5], 5, 4);
  LCD_DisplayDecimals(300, 350, hADC.volt[6], 5, 4);
  LCD_DisplayDecimals(300, 375, hADC.volt[7], 5, 4);
  
  if (ifButtonPressed(&hButtonResetAD7606))
  {
    // Reset AD7606
	AD7606_RST_LOW;
	HAL_Delay(1);
	AD7606_RST_HIGH;
	HAL_Delay(1);
	AD7606_RST_LOW;
  }
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_CustomizedIMU(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonSelectHipIMU);
  ButtonUpdate(&hButtonSelectKneeIMU);
  ButtonUpdate(&hButtonDataLogStart);
  ButtonUpdate(&hButtonDataLogEnd);
  ButtonUpdate(&hButtonAveragingStart);
  ButtonUpdate(&hButtonAveragingStop);
  
  LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(300, 75, hCustomizedIMUPtr->grvNorm.f, 5, 4);
  LCD_DisplayDecimals(300, 100, hCustomizedIMUPtr->xGrv.f, 5, 4);
  LCD_DisplayDecimals(300, 125, hCustomizedIMUPtr->yGrv.f, 5, 4);
  LCD_DisplayDecimals(300, 150, hCustomizedIMUPtr->zGrv.f, 5, 4);
  LCD_DisplayDecimals(300, 175, hCustomizedIMUPtr->xAcc.f, 5, 4);
  LCD_DisplayDecimals(300, 200, hCustomizedIMUPtr->yAcc.f, 5, 4);
  LCD_DisplayDecimals(300, 225, hCustomizedIMUPtr->zAcc.f, 5, 4);
  LCD_DisplayNumber(300, 250, hCustomizedIMUPtr->bno055CalibStatus, 3);
  LCD_SetColor(LCD_RED);
  LCD_DisplayDecimals(300, 0, hCustomizedIMUPtr->xLiAcc.f, 5, 4);
  LCD_DisplayDecimals(300, 25, hCustomizedIMUPtr->yLiAcc.f, 5, 4);
  LCD_DisplayDecimals(300, 50, hCustomizedIMUPtr->zLiAcc.f, 5, 4);
  
  
  if (ifButtonPressed(&hButtonSelectHipIMU))
    hCustomizedIMUPtr = &hIMUHip;
  if (ifButtonPressed(&hButtonSelectKneeIMU))
    hCustomizedIMUPtr = &hIMUKnee;
  if (ifButtonPressed(&hButtonDataLogStart))
    USB_DataLogStart();
  if (ifButtonPressed(&hButtonDataLogEnd))
    USB_DataLogEnd();
  if (ifButtonPressed(&hButtonAveragingStart))
  {
    Averager_Start(&hCustomizedIMUPtr->xAccAvg, hCustomizedIMUPtr->xAcc.f);
    Averager_Start(&hCustomizedIMUPtr->yAccAvg, hCustomizedIMUPtr->yAcc.f);
    Averager_Start(&hCustomizedIMUPtr->zAccAvg, hCustomizedIMUPtr->zAcc.f);
  }
  if (ifButtonPressed(&hButtonAveragingStop))
  {
    Averager_Init(&hCustomizedIMUPtr->xAccAvg);
    Averager_Init(&hCustomizedIMUPtr->yAccAvg);
    Averager_Init(&hCustomizedIMUPtr->zAccAvg);
  }
  
    
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_CustomizedIMU_Init(void)
{
  hUI.task = UI_MAIN_TASK_IMU_MONITOR;
  USB_SetNewDataSlotLen(sizeof(dataSlots_Exoskeleton_Common)/4);
  
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonSelectHipIMU = Button_Create(10, 300, 130, 40, "Hip IMU", LCD_WHITE, LCD_RED);
  hButtonSelectKneeIMU = Button_Create(10, 350, 130, 40, "Knee IMU", LCD_WHITE, LCD_RED);
  hButtonDataLogStart = Button_Create(150, 300, 180, 40, "Datalog Start", LCD_WHITE, LCD_RED);
  hButtonDataLogEnd = Button_Create(150, 350, 180, 40, "Datalog End", LCD_WHITE, LCD_RED);
  hButtonAveragingStart = Button_Create(10, 400, 220, 40, "Acc Avg start", LCD_WHITE, LCD_RED);
  hButtonAveragingStop = Button_Create(250, 400, 220, 40, "Acc Avg Stop ", LCD_WHITE, LCD_RED);
  
  LCD_DisplayString(10, 75, "Gravity vector norm: ");
  LCD_DisplayString(10, 100, "Gravity vector X: ");
  LCD_DisplayString(10, 125, "Gravity vector Y: ");
  LCD_DisplayString(10, 150, "Gravity vector Z: ");
  LCD_DisplayString(10, 175, "Acceleration X:   ");
  LCD_DisplayString(10, 200, "Acceleration Y:   ");
  LCD_DisplayString(10, 225, "Acceleration Z:   ");
  LCD_DisplayString(10, 250, "Calib Status:     ");
  LCD_DisplayString(80, 0, "Linear Acc:");
  LCD_DisplayString(80, 25, "Linear Acc:");
  LCD_DisplayString(80, 50, "Linear Acc:");
}

void UI_Page_LinKongKeJiTesting(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonMotorEnable);
	ButtonUpdate(&hButtonStop);
	ButtonUpdate(&hButtonVelocityControl);
	ButtonUpdate(&hButtonPositionControl);
	ButtonUpdate(&hButtonCurrentControl);
	ButtonUpdate(&hButtonMotorZeroing);
	ButtonUpdate(&hButtonReadAngle);
  ButtonUpdate(&hButtonDataLogStart);
  ButtonUpdate(&hButtonDataLogEnd);
  ButtonUpdate(&hButtonMotorProfilingStart);
  ButtonUpdate(&hButtonMotorProfilingEnd);
  ButtonUpdate(&hButtonEnableVelocityDampingCompensation);
  ButtonUpdate(&hButtonDisableVelocityDampingCompensation);
  PotentialmeterUpdate(&hPotVelComFactor);
	JoystickUpdate(&hJoyLKTECHTesting);
  
  
  if (ifButtonPressed(&hButtonMotorEnable))
	{
		LETECH_MG_Enable(hLKTechTestingMotorPtr);
		hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_ENABLE;
	}
  if (ifButtonPressed(&hButtonStop))
	{
		LETECH_MG_Shutdown(hLKTechTestingMotorPtr);
		hLKTechTestingMotorPtr->task = LKTECH_MG_CAN_BUS_TASK_NONE;
	}
  if (ifButtonPressed(&hButtonVelocityControl))
  {
		hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL;
		hLKTechTestingMotorPtr->velocityControlSet.f = 0.0f;
  }
  if (ifButtonPressed(&hButtonPositionControl))
  {
		hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT;
		hLKTechTestingMotorPtr->positionControlIncrementSet.f = 0.0f;
  }
  if (ifButtonPressed(&hButtonCurrentControl))
  {
		hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL;
		hLKTechTestingMotorPtr->currentControlSet.f = 0.0f;
  }
  if (ifButtonPressed(&hButtonMotorZeroing))
  {
		hLKTechTestingMotorPtr->task = LKTECH_MG_CAN_BUS_TASK_NONE;
		LETECH_MG_Shutdown(hLKTechTestingMotorPtr);
		LETECH_MG_ZeroingByCurrentPosition(hLKTechTestingMotorPtr);;
  }
	if (ifButtonPressed(&hButtonReadAngle))
	{
		ifKeepReadingAngle = !ifKeepReadingAngle;
		hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_READ_ANGLE_SINGLE_TURN;
	}
  if (ifButtonPressed(&hButtonDataLogStart))
    USB_DataLogStart();
  if (ifButtonPressed(&hButtonDataLogEnd))
    USB_DataLogEnd();
  if (ifButtonPressed(&hButtonMotorProfilingStart))
  {
    hLKTechTestingMotorPtr->task = LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL;
		hLKTechTestingMotorPtr->currentControlSet.f += 5.0f / hLKTechTestingMotorPtr->kt;
  }
  if (ifButtonPressed(&hButtonMotorProfilingEnd))
  {
    LETECH_MG_Shutdown(hLKTechTestingMotorPtr);
		hLKTechTestingMotorPtr->task = LKTECH_MG_CAN_BUS_TASK_NONE;
    hLKTechTestingMotorPtr->currentControlSet.f = 0.0f;
  }
  if (ifButtonPressed(&hButtonEnableVelocityDampingCompensation))
    ifVelDampingCompensation = 1;
  if (ifButtonPressed(&hButtonDisableVelocityDampingCompensation))
    ifVelDampingCompensation = 0;
	
	if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL)
		hLKTechTestingMotorPtr->velocityControlSet.f = LKTECHJoyStickValue * 360.0f;
	else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT)
		hLKTechTestingMotorPtr->positionControlIncrementSet.f = LKTECHJoyStickValue * 10.0f;
	else if (hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL)
		hLKTechTestingMotorPtr->currentControlSet.f = LKTECHJoyStickValue * 2.0f;
	else if (ifKeepReadingAngle && hLKTechTestingMotorPtr->task == LETECH_MG_CAN_BUS_TASK_READ_ANGLE_SINGLE_TURN)
		LETECH_MG_ReadAngleSingleTurn(hLKTechTestingMotorPtr);
	
	LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
	LCD_DisplayDecimals(200, 200, hLKTechTestingMotorPtr->angle.f, 4, 1);
	LCD_DisplayDecimals(200, 230, hLKTechTestingMotorPtr->speedDeg.f, 4, 1);
	LCD_DisplayDecimals(200, 260, hLKTechTestingMotorPtr->torque.f, 6, 4);
  LCD_DisplayDecimals(200, 290, hLKTechTestingMotorPtr->temperature.f, 4, 1);
  LCD_DisplayDecimals(50, 480, velComFactor, 4,1);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_LinKongKeJiTesting_Init(void)
{
  hUI.task = UI_MAIN_TASK_LINKONGKEJI_MG_MOTOR;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorEnable = Button_Create(5, 80, 100, 40, "Enable", LCD_WHITE, LCD_RED);
	hButtonStop = Button_Create(5, 160, 100, 150, "STOP", LCD_YELLOW, LCD_RED);
	hButtonVelocityControl = Button_Create(120, 50, 180, 40, "Vel Control", LCD_GREEN, LCD_RED);
	hButtonPositionControl = Button_Create(120, 100, 180, 40, "Pos Control", LCD_GREEN, LCD_RED);
	hButtonCurrentControl = Button_Create(120, 150, 180, 40, "Cur Control", LCD_GREEN, LCD_RED);
  hButtonMotorZeroing = Button_Create(330, 50, 140, 40, "Zeroing", LCD_GREEN, LCD_RED);
	hButtonReadAngle = Button_Create(330, 100, 140, 70, "Read Angle", LCD_GREEN, LCD_RED);
  hButtonDataLogStart = Button_Create(330, 200, 100, 40, "DataLogS", LIGHT_MAGENTA, LCD_RED);
  hButtonDataLogEnd = Button_Create(330, 250, 100, 40, "DataLogE", LCD_GREEN, LCD_RED);
  hButtonMotorProfilingStart = Button_Create(280, 300, 100, 40, "ProfSt", LCD_GREEN, LCD_RED);
  hButtonMotorProfilingEnd = Button_Create(280, 350, 100, 40, "ProfEd", LCD_GREEN, LCD_RED);
  hButtonEnableVelocityDampingCompensation = Button_Create(5, 400, 130, 40, "VelComEna", LCD_GREEN, LCD_RED);
  hButtonDisableVelocityDampingCompensation = Button_Create(150, 400, 130, 40, "VelComDisa", LCD_GREEN, LCD_RED);
	hJoyLKTECHTesting = Joystick_Create(310, 630, 130, LCD_WHITE, LCD_BLACK, 50, 180, &LKTECHJoyStickValue, &LKTECHBlank, 0.0f, 1.0f, 1.0f, -1.0f);
  
  hPotVelComFactor = Potentialmeter_Create(50, 480, 40, 300, 100, 80, \
                                    LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 5.0f, 0.0f, &velComFactor);
  
  
  USB_SetNewDataSlotLen(sizeof(dataSlots_LKTECH_MG_MotorTest)/4);
	
	LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
	LCD_DisplayString(120, 200, "Pos:");
	LCD_DisplayString(120, 230, "Vel:");
	LCD_DisplayString(120, 260, "Toq:");
  LCD_DisplayString(120, 290, "Tem:");
  
  hLKTechTestingMotorPtr = &hLKTECH;
}


void UI_Page_TkCalibration(void)
{
  ButtonUpdate(&hButtonGoBack);
  ButtonUpdate(&hButtonMotorStart);
  ButtonUpdate(&hButtonMotorStop);
  ButtonUpdate(&hButtonMotorZeroing);
  ButtonUpdate(&hButtonTkCalculateAverageIq);
  PotentialmeterUpdate(&hTMotorManualControlPot_pos);
  PotentialmeterUpdate(&hTMotorManualControlPot_kp);
  PotentialmeterUpdate(&hTMotorManualControlPot_kd);
  
  if(ifButtonPressed(&hButtonMotorStart))
  {
    AK10_9_MITMode_EnableMotor(hMotorPtrManualControl);
    torqueConstantCalibrationIfMotorStarted = 1;
  }
  if(ifButtonPressed(&hButtonMotorStop))
  {
    AK10_9_MITMode_DisableMotor(hMotorPtrManualControl);
    torqueConstantCalibrationIfMotorStarted = 0;
  }
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_MITMode_Zeroing(hMotorPtrManualControl);
  
  if (hAverageTorqueConstantCalibration.ifStarted)
    Averager_Update(&hAverageTorqueConstantCalibration, hMotorPtrManualControl->realCurrent.f);
  if(ifButtonPressed(&hButtonTkCalculateAverageIq))
  {
    Averager_Init(&hAverageTorqueConstantCalibration);
    Averager_Start(&hAverageTorqueConstantCalibration, hMotorPtrManualControl->realCurrent.f);
  }
  
  if (torqueConstantCalibrationIfMotorStarted)
    AK10_9_MITModeControl_Deg(hMotorPtrManualControl, torqueConstantCalibrationMotorSetP, 0.0f, 200.0f, 2.0f, 0.0f);
  
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);

  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  LCD_DisplayDecimals(340, 510, (double)torqueConstantCalibrationMotorKp, 3, 0);
  LCD_DisplayDecimals(420, 510, (double)torqueConstantCalibrationMotorKd, 3, 2);
  LCD_DisplayDecimals(180, 80, (double)torqueConstantCalibrationMotorSetP, 3, 1);
  LCD_DisplayDecimals(280, 100, hMotorPtrManualControl->realPosition.f, 9, 4);
  LCD_DisplayDecimals(280, 130, hMotorPtrManualControl->realVelocityPresent.f, 4, 1);
  LCD_DisplayDecimals(280, 160, hMotorPtrManualControl->realCurrent.f, 10, 7);
  LCD_SetColor(LCD_RED);
  LCD_DisplayDecimals(280, 270, hAverageTorqueConstantCalibration.avg.f, 10, 7);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_TkCalibration_Init(void)
{
  hUI.task = UI_MAIN_TASK_KT_CALIBRATION;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorStart = Button_Create(0, 420, 100, 40, "START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(0, 80, 150, 250, "STOP", LCD_RED, LCD_YELLOW);
  hButtonMotorZeroing = Button_Create(0, 500, 100, 40, "Zeroing", LCD_BLUE, LCD_RED);
  hButtonTkCalculateAverageIq = Button_Create(280, 300, 160, 80, "Averaging Iq", LCD_YELLOW, LCD_RED);
  
  
  hTMotorManualControlPot_pos = Potentialmeter_Create(180, 80, 30, 700, 100, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -10.0f, 40.0f, 0.0f, &torqueConstantCalibrationMotorSetP);
  hTMotorManualControlPot_kp = Potentialmeter_Create(340, 510, 30, 200, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 500.0f, 0.0f, &torqueConstantCalibrationMotorKp);
  hTMotorManualControlPot_kd = Potentialmeter_Create(420, 510, 30, 200, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 5.0f, 0.0f, &torqueConstantCalibrationMotorKd);
  
  LCD_DisplayString(340, 480, "kp");
  LCD_DisplayString(420, 480, "kd");
  LCD_DisplayString(250, 100, "P:");
  LCD_DisplayString(250, 130, "V:");
  LCD_DisplayString(250, 160, "I:");
  
  torqueConstantCalibrationMotorKp = 0.0f;
  torqueConstantCalibrationMotorKd = 0.0f;
  torqueConstantCalibrationMotorSetP = 0.0f;
  Averager_Init(&hAverageTorqueConstantCalibration);
  hMotorPtrManualControl = &hAKMotorRightKnee;
  torqueConstantCalibrationIfMotorStarted = 0;
}
