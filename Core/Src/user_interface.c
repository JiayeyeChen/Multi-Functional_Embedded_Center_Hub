#include "user_interface.h"
#include "main.h"
#include <math.h>

UIHandle hUI;

PageHandle UIPage_Home1;
/* Common Stuff */
ButtonHandle hButtonGoBack, hButtonDataLogStart, hButtonDataLogEnd, \
             hButtonMotorProfilingStart, hButtonMotorProfilingEnd, \
             hButtonMotorZeroing, hButtonMotorStart, hButtonMotorStop, \
             hButtonStart, hButtonStop, hButtonOn, hButtonOff;
//////////////////
/* Exoskeleton User Interface */
PageHandle UIPage_LowerLimb_Exoskeleton, UIPage_LowerLimb_SystemID, UIPage_LowerLimb_GravityCompensation, \
           UIPage_LowerLimb_ParameterAdjusting, UIPage_LowerLimb_MuscularTorqueMonitor;
ButtonHandle hButtonPageExoskeletonInterface, hButtonPageSystemID, hButtonPageGravityCompensation, \
             hButtonPageExoskeletonParameterPanel, hButtonPageExoskeletonMuscularTorqueMonitor, \
             hButtonSystemIDJointMovementStart, hButtonHipMotorZeroing, \
             hButtonKneeMotorZeroing, hButtonProfilingTimeIncrease, hButtonProfilingTimeDecrease, \
             hButtonMotorEnable, hButtonMotorDisable;
             
LinearPotentialmeterHandle hPotKneeProfilingFreq, hPotKneeProfilingAmp, hPotHipProfilingFreq, hPotHipProfilingAmp, \
                           hPotGravityCompensationHipThrottle, hPotGravityCompensationKneeThrottle, \
                           hPotParameterAdjust_L1, hPotParameterAdjust_J1, hPotParameterAdjust_X1, \
                           hPotParameterAdjust_J2, hPotParameterAdjust_X2;
////////////////////////////////
/* AK10-9 Manual Control */
PageHandle UIPage_AK10_9_ManualControlCubeMarsFWServoMode, UIPage_AK10_9_ManualControlCubeMarsFWMITMode, UIPage_AK10_9_ManualControlFirmwareSelection, UIPage_AK10_9_ManualControlDMFW;
ButtonHandle hButtonPageAK10_9ManualControl, hButtonManualControlMode, hButtonMotorSelectRightHip, \
                                             hButtonMotorSelectRightKnee, \
                                             hButtonAK10_9_ManualControlCubeMarsFWServoMode, \
                                             hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode,\
                                             hButtonAK10_9_ManualControlDMFW;
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
PageHandle UIPage_TMotor_Acceleration_Observer_Project;
ButtonHandle hButtonPageTMotorAccelerationObserverProject;
LinearPotentialmeterHandle  hPotTMotorProfilingFrequency;
//////////////////////////////////////
/* Briter Encoder Monitor */
PageHandle UIPage_BriterEncoder;
ButtonHandle hButtonPageBriterEncoder, hButtonBriterEncoderZeroing, \
             hButtonBriterEncoderSetCounterClockWiseDirection, hButtonBriterEncoderSetCanID, \
             hButtonBriterEncoderSet1MHzCanRate, hButtonBriterEncoderSelectEncoder, \
             hButtonBriterEncoderIfRead;
////////////////////////////
/* AD7606 Monitor */
PageHandle UIPage_ADC_Monitor;
ButtonHandle hButtonPageADCMonitor, hButtonResetAD7606;
////////////////////

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

uint8_t ButtonScan(ButtonHandle* hbutton)
{
  for (uint8_t i = 0; i <= 4; i++)
  {
    if ((touchInfo.xVertical[i] > hbutton->pos.x && touchInfo.xVertical[i] < (hbutton->pos.x + hbutton->pos.xLen)) &&\
        (touchInfo.yVertical[i] > hbutton->pos.y && touchInfo.yVertical[i] < (hbutton->pos.y + hbutton->pos.yLen)))
    {
      hbutton->ifPressed = 1;
      return 1;
    }

  }
  hbutton->ifPressed = 0;
  return 0;
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
  hpotentialmeter.pos.ySlider = y + yLen - sliderLen - (yLen - sliderLen)*(uint16_t)(startVal - minVal)/(uint16_t)(maxVal - minVal);
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
  
  UIPage_AK10_9_ManualControlFirmwareSelection.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControlFirmwareSelection.Page = UI_Page_AK10_9_ManualControlFirmwareSelection;
  UIPage_AK10_9_ManualControlFirmwareSelection.PageInit = UI_Page_AK10_9_ManualControlFirmwareSelection_Init;
  
  UIPage_AK10_9_ManualControlDMFW.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControlDMFW.Page = UI_Page_AK10_9_ManualControlDMFW;
  UIPage_AK10_9_ManualControlDMFW.PageInit = UI_Page_AK10_9_ManualControlDMFW_Init;
  
  UIPage_BNO055_Monitor.ifPageInitialized = 0;
  UIPage_BNO055_Monitor.Page = UI_Page_BNO055_Monitor;
  UIPage_BNO055_Monitor.PageInit = UI_Page_BNO055_Monitor_Init;
  
  UIPage_TMotor_Acceleration_Observer_Project.ifPageInitialized = 0;
  UIPage_TMotor_Acceleration_Observer_Project.Page = UI_Page_TMotor_Acceleration_Observer_Project;
  UIPage_TMotor_Acceleration_Observer_Project.PageInit = UI_Page_TMotor_Acceleration_Observer_Project_Init;
  
  UIPage_ADC_Monitor.ifPageInitialized = 0;
  UIPage_ADC_Monitor.Page = UI_Page_ADC_Monitor;
  UIPage_ADC_Monitor.PageInit = UI_Page_ADC_Monitor_Init;
  
  UIPage_BriterEncoder.ifPageInitialized = 0;
  UIPage_BriterEncoder.Page = UI_Page_BriterEncoder;
  UIPage_BriterEncoder.PageInit = UI_Page_BriterEncoder_Init;
}

JoystickHandle Joystick_Create(uint16_t x, uint16_t y, uint16_t r, char label[])
{
  JoystickHandle hjoy;
  hjoy.pos.x = x;
  hjoy.pos.y = y;
  LCD_SetLayer(0);
  LCD_SetColor(DARK_CYAN);
  LCD_FillCircle(hjoy.pos.x, hjoy.pos.y, hjoy.pos.r);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawCircle(hjoy.pos.x, hjoy.pos.y, hjoy.pos.r);
  LCD_DisplayString(hjoy.pos.x, hjoy.pos.y - 30, (char*)label);
  
  return hjoy;
}

void ButtonRefresh(ButtonHandle* hbutton)
{
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonPageSystemID);
  ButtonRefresh(&hButtonPageSystemID);
  ButtonScan(&hButtonHipMotorZeroing);
  ButtonRefresh(&hButtonHipMotorZeroing);
  ButtonScan(&hButtonKneeMotorZeroing);
  ButtonRefresh(&hButtonKneeMotorZeroing);
  ButtonScan(&hButtonMotorEnable);
  ButtonRefresh(&hButtonMotorEnable);
  ButtonScan(&hButtonMotorDisable);
  ButtonRefresh(&hButtonMotorDisable);
  ButtonScan(&hButtonPageGravityCompensation);
  ButtonRefresh(&hButtonPageGravityCompensation);
  ButtonScan(&hButtonPageExoskeletonParameterPanel);
  ButtonRefresh(&hButtonPageExoskeletonParameterPanel);
  ButtonScan(&hButtonPageExoskeletonMuscularTorqueMonitor);
  ButtonRefresh(&hButtonPageExoskeletonMuscularTorqueMonitor);
  
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
    AK10_9_DMFW_Zeroing(&hAKMotorRightHip);
  else if(ifButtonPressed(&hButtonKneeMotorZeroing))
    AK10_9_MITMode_Zeroing(&hAKMotorRightKnee);
  else if (ifButtonPressed(&hButtonMotorEnable))
  {
    AK10_9_DMFW_EnableMotor(&hAKMotorRightHip);
    AK10_9_MITMode_EnableMotor(&hAKMotorRightKnee);
  }
  else if (ifButtonPressed(&hButtonMotorDisable))
  {
    AK10_9_DMFW_DisableMotor(&hAKMotorRightHip);
    AK10_9_MITMode_DisableMotor(&hAKMotorRightKnee);
  }
  else if (ifButtonPressed(&hButtonPageGravityCompensation))
    UI_Page_Change_To(&UIPage_LowerLimb_GravityCompensation);
  else if (ifButtonPressed(&hButtonPageExoskeletonParameterPanel))
    UI_Page_Change_To(&UIPage_LowerLimb_ParameterAdjusting);
  else if (ifButtonPressed(&hButtonPageExoskeletonMuscularTorqueMonitor))
    UI_Page_Change_To(&UIPage_LowerLimb_MuscularTorqueMonitor);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_LowerLimb_Exoskeleton_Init(void)
{
  hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_FREE;
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonPageSystemID = Button_Create(100, 50, 280, 40, "System Identification", LCD_WHITE, LCD_RED);
  hButtonHipMotorZeroing = Button_Create(100, 150, 280, 40, "Hip Joint Zeroing", LCD_WHITE, LCD_RED);
  hButtonKneeMotorZeroing = Button_Create(100, 200, 280, 40, "Knee Joint Zeroing", LCD_WHITE, LCD_RED);
  hButtonMotorEnable = Button_Create(100, 350, 280, 60, "Motor Enable", LIGHT_YELLOW, LCD_RED);
  hButtonMotorDisable = Button_Create(100, 420, 280, 100, "Motor Disable", LIGHT_YELLOW, LCD_RED);
  hButtonPageGravityCompensation = Button_Create(100, 530, 280, 60, "Gravity Compensation", LIGHT_GREEN, LCD_RED);
  hButtonPageExoskeletonParameterPanel = Button_Create(150, 250, 280, 60, "Parameter Adjusting", LIGHT_GREEN, LCD_RED);
  hButtonPageExoskeletonMuscularTorqueMonitor = Button_Create(100, 600, 280, 60, "Muscular Torque", LIGHT_GREEN, LCD_RED);
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonStart);
  ButtonRefresh(&hButtonStart);
  ButtonScan(&hButtonSystemIDJointMovementStart);
  ButtonRefresh(&hButtonSystemIDJointMovementStart);
  ButtonScan(&hButtonStop);
  ButtonRefresh(&hButtonStop);
  ButtonScan(&hButtonProfilingTimeIncrease);
  ButtonRefresh(&hButtonProfilingTimeIncrease);
  ButtonScan(&hButtonProfilingTimeDecrease);
  ButtonRefresh(&hButtonProfilingTimeDecrease);
  
  
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
  if (hAKMotorRightHip_old.status == AK10_9_Online)
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
    AK10_9_DMFW_EnableMotor(&hAKMotorRightHip);
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
    AK10_9_DMFW_DisableMotor(&hAKMotorRightHip);
    hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_FREE;
    hUSB.datalogTask = DATALOG_TASK_FREE;
  }
  if (ifButtonPressed(&hButtonProfilingTimeIncrease))
  {
    if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.kneeProfilingTime += 10000;
    else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.hipProfilingTime += 10000;
    LIMIT_MIN_MAX(hSystemID.kneeProfilingTime, 0, 600000);
    LIMIT_MIN_MAX(hSystemID.hipProfilingTime, 0, 600000);
  }
  else if (ifButtonPressed(&hButtonProfilingTimeDecrease))
  {
    if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.kneeProfilingTime -= 10000;
    else if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START)
      hSystemID.hipProfilingTime -= 10000;
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonStart);
  ButtonRefresh(&hButtonStart);
  ButtonScan(&hButtonStop);
  ButtonRefresh(&hButtonStop);
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
    AK10_9_DMFW_DisableMotor(&hAKMotorRightHip);
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonOn);
  ButtonRefresh(&hButtonOn);
  ButtonScan(&hButtonOff);
  ButtonRefresh(&hButtonOff);
  ButtonScan(&hButtonDataLogStart);
  ButtonRefresh(&hButtonDataLogStart);
  ButtonScan(&hButtonDataLogEnd);
  ButtonRefresh(&hButtonDataLogEnd);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(300, 0, hExoskeleton.hmusculartorque->muscularTorqueHip.f, 5, 1);
  LCD_DisplayDecimals(300, 25, hExoskeleton.hmusculartorque->muscularTorqueKnee.f, 5, 1);
  LCD_DisplayString(200, 0, "Hip/Nm:  ");
  LCD_DisplayString(200, 25,"Knee/Nm: ");
  
  if (ifButtonPressed(&hButtonOn))
    hExoskeleton.hmusculartorque->ifEstimating = 1;
  else if (ifButtonPressed(&hButtonOff))
    hExoskeleton.hmusculartorque->ifEstimating = 0;
  else if (ifButtonPressed(&hButtonDataLogStart))
  {
  }
  else if (ifButtonPressed(&hButtonDataLogEnd))
  {
  }
  
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
}
void UI_Page_LowerLimb_Exoskeleton_MuscularTorqueMonitor_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonOn = Button_Create(100, 100, 100, 70, "On", LCD_WHITE, LCD_RED);
  hButtonOff = Button_Create(100, 200, 100, 70, "Off", LCD_WHITE, LCD_RED);
  hButtonDataLogStart = Button_Create(100, 300, 200, 70, "Datalog Start", LCD_WHITE, LCD_RED);
  hButtonDataLogEnd = Button_Create(100, 400, 200, 70, "Datalog End", LCD_WHITE, LCD_RED);
  
  LCD_DisplayString(200, 0, "Hip/Nm:  ");
  LCD_DisplayString(200, 25,"Knee/Nm: ");
}

void UI_Page_Home1(void)
{
  LCD_SetFont(&Font32); 
  LCD_DisplayString(140, 30, "Welcome Jiaye");
  LCD_SetFont(&Font24);
  ButtonScan(&hButtonPageExoskeletonInterface);
  ButtonRefresh(&hButtonPageExoskeletonInterface);
  ButtonScan(&hButtonPageAK10_9ManualControl);
  ButtonRefresh(&hButtonPageAK10_9ManualControl);
  ButtonScan(&hButtonPageBNO055_Monitor);
  ButtonRefresh(&hButtonPageBNO055_Monitor);
  ButtonScan(&hButtonPageTMotorAccelerationObserverProject);
  ButtonRefresh(&hButtonPageTMotorAccelerationObserverProject);
  ButtonScan(&hButtonPageADCMonitor);
  ButtonRefresh(&hButtonPageADCMonitor);
  ButtonScan(&hButtonPageBriterEncoder);
  ButtonRefresh(&hButtonPageBriterEncoder);
  
  if (ifButtonPressed(&hButtonPageExoskeletonInterface))
    UI_Page_Change_To(&UIPage_LowerLimb_Exoskeleton);
  if (ifButtonPressed(&hButtonPageAK10_9ManualControl))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControlFirmwareSelection);
  if (ifButtonPressed(&hButtonPageBNO055_Monitor))
    UI_Page_Change_To(&UIPage_BNO055_Monitor);
  if (ifButtonPressed(&hButtonPageTMotorAccelerationObserverProject))
    UI_Page_Change_To(&UIPage_TMotor_Acceleration_Observer_Project);
  if (ifButtonPressed(&hButtonPageADCMonitor))
    UI_Page_Change_To(&UIPage_ADC_Monitor);
  if (ifButtonPressed(&hButtonPageBriterEncoder))
    UI_Page_Change_To(&UIPage_BriterEncoder);
}
void UI_Page_Home1_Init(void)
{
  hButtonPageExoskeletonInterface = Button_Create(100, 100, 300, 40, "Lower Limb Exoskeleton", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAK10_9ManualControl = Button_Create(80, 150, 360, 40, "AK10-9 V2.0 Manual Control", LIGHT_MAGENTA, LCD_RED);
  hButtonPageBNO055_Monitor = Button_Create(150, 250, 200, 40, "BNO055 Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageTMotorAccelerationObserverProject = Button_Create(10, 300, 450, 40, "TMotor Acceleration Observer Project", LIGHT_MAGENTA, LCD_RED);
  hButtonPageADCMonitor = Button_Create(150, 350, 200, 40, "ADC Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageBriterEncoder = Button_Create(150, 400, 200, 40, "Briter Encoder", LIGHT_MAGENTA, LCD_RED);
}

void UI_Page_AK10_9_ManualControlCubeMarsFWServoMode(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonMotorStart);
  ButtonScan(&hButtonMotorStop);
  ButtonScan(&hButtonMotorZeroing);
  ButtonScan(&hButtonManualControlMode);
  ButtonScan(&hButtonMotorSelectRightHip);
  ButtonScan(&hButtonMotorSelectRightKnee);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonMotorStart);
  ButtonRefresh(&hButtonMotorStop);
  ButtonRefresh(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonManualControlMode);
  ButtonRefresh(&hButtonMotorSelectRightHip);
  ButtonRefresh(&hButtonMotorSelectRightKnee);
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
    hMotorPtrManualControl = &hAKMotorRightHip_old;
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
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonMotorStart);
  ButtonScan(&hButtonMotorStop);
  ButtonScan(&hButtonMotorZeroing);
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonMotorSelectRightHip);
  ButtonScan(&hButtonMotorSelectRightKnee);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonMotorStart);
  ButtonRefresh(&hButtonMotorStop);
  ButtonRefresh(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonMotorSelectRightHip);
  ButtonRefresh(&hButtonMotorSelectRightKnee);
  PotentialmeterUpdate(&hTMotorManualControlPot_pos);
  PotentialmeterUpdate(&hTMotorManualControlPot_vel);
  PotentialmeterUpdate(&hTMotorManualControlPot_cur);
  PotentialmeterUpdate(&hTMotorManualControlPot_kp);
  PotentialmeterUpdate(&hTMotorManualControlPot_kd);
  
  if (ifButtonPressed(&hButtonMotorSelectRightHip))
  {
    ifManualControlStarted = 0;
    hMotorPtrManualControl = &hAKMotorRightHip_old;
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



void UI_Page_AK10_9_ManualControlDMFW_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorStart = Button_Create(0, 420, 100, 40, "START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(0, 80, 150, 250, "STOP", LCD_RED, LCD_YELLOW);
  hButtonMotorZeroing = Button_Create(0, 620, 200, 40, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonManualControlMode = Button_Create(0, 470, 160, 40, "Control Mode", LCD_WHITE, LCD_RED);
  hTMotorManualControlPot_pos = Potentialmeter_Create(250, 80, 30, 400, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -180.0f, 180.0f, 0.0f, &manualControlValue_pos);
  hTMotorManualControlPot_vel = Potentialmeter_Create(340, 80, 30, 400, 60, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, -360.0f, 360.0f, 0.0f, &manualControlValue_vel);
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
void UI_Page_AK10_9_ManualControlDMFW(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonMotorStart);
  ButtonScan(&hButtonMotorStop);
  ButtonScan(&hButtonMotorZeroing);
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonManualControlMode);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonMotorStart);
  ButtonRefresh(&hButtonMotorStop);
  ButtonRefresh(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonManualControlMode);
  PotentialmeterUpdate(&hTMotorManualControlPot_pos);
  PotentialmeterUpdate(&hTMotorManualControlPot_vel);
  PotentialmeterUpdate(&hTMotorManualControlPot_cur);
  PotentialmeterUpdate(&hTMotorManualControlPot_kp);
  PotentialmeterUpdate(&hTMotorManualControlPot_kd);
  
  if(ifButtonPressed(&hButtonMotorZeroing))
  {
    AK10_9_DMFW_Zeroing(&hAKMotorRightHip);
  }
  if(ifButtonPressed(&hButtonMotorStart))
  {
    ifManualControlStarted = 1;
    AK10_9_DMFW_EnableMotor(&hAKMotorRightHip);
  }
  if(ifButtonPressed(&hButtonMotorStop))
  {
    AK10_9_DMFW_DisableMotor(&hAKMotorRightHip);
    ifManualControlStarted = 0;
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
  }
  if(ifButtonPressed(&hButtonManualControlMode))
  {
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_pos, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_vel, 0.0f);
    PotentialmeterSliderGoTo(&hTMotorManualControlPot_cur, 0.0f);
    ifManualControlStarted = 0;
    hAKMotorRightHip.controlMode++;
    if ((uint32_t)hAKMotorRightHip.controlMode > 2)
      hAKMotorRightHip.controlMode = 0;
  }
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_MIT)
  {
    LCD_DisplayString(0, 340, "MIT Mode");
  }
  else if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_POSITION)
  {
    LCD_DisplayString(0, 340, "Position Control");
  }
  else if (hAKMotorRightHip.controlMode == AK10_9_DM_FW_MODE_VELOCITY)
  {
    LCD_DisplayString(0, 340, "Velocity Control");
  }
  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  LCD_DisplayDecimals(400, 735, (double)hAKMotorRightHip.realTorque.f, 3, 2);
  LCD_DisplayDecimals(150, 710, (double)hAKMotorRightHip.realPositionDeg.f, 6, 3);
  LCD_DisplayDecimals(150, 735, (double)hAKMotorRightHip.realVelocityPresent.f, 6, 3);
  LCD_DisplayDecimals(150, 760, (double)hAKMotorRightHip.realCurrent.f, 6, 3);
  LCD_DisplayDecimals(250, 710, (double)hAKMotorRightHip.setPos.f, 6, 3);
  LCD_DisplayDecimals(250, 735, (double)hAKMotorRightHip.setVel.f, 6, 3);
  LCD_DisplayDecimals(250, 760, (double)hAKMotorRightHip.setIq.f, 6, 3);
  LCD_DisplayDecimals(250, 80, (double)manualControlValue_pos, 3, 1);
  LCD_DisplayDecimals(340, 80, (double)manualControlValue_vel, 3, 1);
  LCD_DisplayDecimals(420, 80, (double)manualControlValue_cur, 3, 1);
  LCD_DisplayDecimals(340, 510, (double)manualControlValue_kp, 3, 0);
  LCD_DisplayDecimals(420, 510, (double)manualControlValue_kd, 3, 2);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_AK10_9_ManualControlFirmwareSelection(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonAK10_9_ManualControlCubeMarsFWServoMode);
  ButtonRefresh(&hButtonAK10_9_ManualControlCubeMarsFWServoMode);
  ButtonScan(&hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode);
  ButtonRefresh(&hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode);
  ButtonScan(&hButtonAK10_9_ManualControlDMFW);
  ButtonRefresh(&hButtonAK10_9_ManualControlDMFW);
  
  
  
  if (ifButtonPressed(&hButtonAK10_9_ManualControlCubeMarsFWServoMode))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControlCubeMarsFWServoMode);
  else if (ifButtonPressed(&hButtonAK10_9_ManualControlDMFW))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControlDMFW);
  else if (ifButtonPressed(&hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControlCubeMarsFWMITMode);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_AK10_9_ManualControlFirmwareSelection_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonAK10_9_ManualControlCubeMarsFWServoMode = Button_Create(10, 300, 400, 60, "CubeMars Firmware Servo Mode", LCD_WHITE, LCD_RED);
  hButtonAK10_9_ManualControlCubeMarsFWServoModeMITMode = Button_Create(10, 400, 400, 60, "CubeMars Firmware MIT Mode", LCD_WHITE, LCD_RED);
  hButtonAK10_9_ManualControlDMFW = Button_Create(100, 500, 200, 60, "DM Firmware", LCD_WHITE, LCD_RED);
  
}

void UI_Page_BNO055_Monitor(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonIMUSetModeNDOF);
  ButtonRefresh(&hButtonIMUSetModeNDOF);
  ButtonScan(&hButtonIMUSetModeACCONLY);
  ButtonRefresh(&hButtonIMUSetModeACCONLY);
  ButtonScan(&hButtonIMUSetModeGYROONLY);
  ButtonRefresh(&hButtonIMUSetModeGYROONLY);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(200, 0, (double)hIMURightThigh.rawData.liaccX.b16, 7, 1);
  LCD_DisplayDecimals(200, 50, (double)hIMURightThigh.rawData.liaccY.b16, 7, 1);
  LCD_DisplayDecimals(200, 100, (double)hIMURightThigh.rawData.liaccZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 150, (double)hIMURightThigh.rawData.AccX.b16, 7, 1);
  LCD_DisplayDecimals(200, 200, (double)hIMURightThigh.rawData.AccY.b16, 7, 1);
  LCD_DisplayDecimals(200, 250, (double)hIMURightThigh.rawData.AccZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 300, (double)hIMURightThigh.rawData.gyroX.b16, 7, 1);
  LCD_DisplayDecimals(200, 350, (double)hIMURightThigh.rawData.gyroY.b16, 7, 1);
  LCD_DisplayDecimals(200, 400, (double)hIMURightThigh.rawData.gyroZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 450, (double)hIMURightThigh.rawData.MagX.b16, 7, 1);
  LCD_DisplayDecimals(200, 500, (double)hIMURightThigh.rawData.MagY.b16, 7, 1);
  LCD_DisplayDecimals(200, 550, (double)hIMURightThigh.rawData.MagZ.b16, 7, 1);
  
  if (hIMURightThigh.operationModeENUM == IMU_MODE_NDOF)
    EXOSKELETON_SetIMUMode_9_DOF(&hIMURightThigh);
  else if (hIMURightThigh.operationModeENUM == IMU_MODE_ACCONLY)
    EXOSKELETON_SetIMUMode_ACC_Only(&hIMURightThigh);
  else if (hIMURightThigh.operationModeENUM == IMU_MODE_GYROONLY)
    EXOSKELETON_SetIMUMode_GYRO_Only(&hIMURightThigh);
  
  if (ifButtonPressed(&hButtonIMUSetModeNDOF))
    hIMURightThigh.operationModeENUM = IMU_MODE_NDOF;
  if (ifButtonPressed(&hButtonIMUSetModeACCONLY))
    hIMURightThigh.operationModeENUM = IMU_MODE_ACCONLY;
  if (ifButtonPressed(&hButtonIMUSetModeGYROONLY))
    hIMURightThigh.operationModeENUM = IMU_MODE_GYROONLY;
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_BNO055_Monitor_Init(void)
{
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

void UI_Page_TMotor_Acceleration_Observer_Project(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonDataLogStart);
  ButtonScan(&hButtonDataLogEnd);
  ButtonScan(&hButtonMotorProfilingStart);
  ButtonScan(&hButtonMotorProfilingEnd);
  ButtonScan(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonDataLogEnd);
  ButtonRefresh(&hButtonDataLogStart);
  ButtonRefresh(&hButtonMotorProfilingStart);
  ButtonRefresh(&hButtonMotorProfilingEnd);
  ButtonRefresh(&hButtonMotorZeroing);
  PotentialmeterUpdate(&hPotTMotorProfilingFrequency);
  
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
    AK10_9_ServoMode_Zeroing(&hAKMotorRightHip_old);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hAKMotorRightHip_old.status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  
  LCD_DisplayDecimals(140, 720, (double)hAKMotorRightHip_old.realPosition.f, 10, 4);
  LCD_DisplayDecimals(140, 745, (double)hAKMotorRightHip_old.setPos.f, 10, 4);
  LCD_DisplayDecimals(140, 770, (double)hAKMotorRightHip_old.realVelocityPresent.f, 10, 4);
  LCD_SetColor(DARK_RED);
  LCD_DisplayDecimals(90, 495, (double)hIMURightThigh.rawData.liaccX.b16, 7, 1);
  LCD_DisplayDecimals(90, 520, (double)hIMURightThigh.rawData.liaccY.b16, 7, 1);
  LCD_DisplayDecimals(90, 545, (double)hIMURightThigh.rawData.liaccZ.b16, 7, 1);
  LCD_DisplayDecimals(90, 570, (double)hIMURightThigh.rawData.AccX.b16, 7, 1);
  LCD_DisplayDecimals(90, 595, (double)hIMURightThigh.rawData.AccY.b16, 7, 1);
  LCD_DisplayDecimals(90, 620, (double)hIMURightThigh.rawData.AccZ.b16, 7, 1);
  LCD_DisplayDecimals(90, 645, (double)hIMURightThigh.rawData.gyroX.b16, 7, 1);
  LCD_DisplayDecimals(90, 670, (double)hIMURightThigh.rawData.gyroY.b16, 7, 1);
  LCD_DisplayDecimals(90, 695, (double)hIMURightThigh.rawData.gyroZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 470, (double)tmotorProfilingSinWaveFrequency, 3, 4);
  LCD_DisplayDecimals(230, 445, (double)(hAKMotorRightHip_old.realPosition.f - hAKMotorRightHip_old.setPos.f), 3, 4);
  LCD_SetColor(LCD_BLACK);
  
  ifIMUFeedbackStarted = 1;
  if (ifButtonPressed(&hButtonGoBack))
  {
    UI_Page_Change_To(&UIPage_Home1);
    ifIMUFeedbackStarted = 0;
  }
}
void UI_Page_TMotor_Acceleration_Observer_Project_Init(void)
{
  hButtonDataLogStart = Button_Create(10, 100, 200, 40, "Data Log Start", LIGHT_MAGENTA, LCD_RED);
  hButtonDataLogEnd = Button_Create(10, 150, 200, 40, "Data Log End", LCD_GREEN, LCD_RED);
  hButtonMotorProfilingStart = Button_Create(10, 200, 300, 40, "Motor profiling Start", LIGHT_GREY, LCD_RED);
  hButtonMotorProfilingEnd = Button_Create(10, 250, 300, 40, "Motor profiling Stop", LCD_YELLOW, LCD_RED);
  hButtonMotorZeroing = Button_Create(10, 330, 200, 40, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hPotTMotorProfilingFrequency = Potentialmeter_Create(420, 80, 30, 550, 130, 70, LCD_MAGENTA, LCD_RED, LIGHT_GREY, 0.0f, 1.0f, 0.0f, &tmotorProfilingSinWaveFrequency);
  
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
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonResetAD7606);
  ButtonRefresh(&hButtonResetAD7606);
  
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

void UI_Page_BriterEncoder(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonBriterEncoderZeroing);
  ButtonRefresh(&hButtonBriterEncoderZeroing);
  ButtonScan(&hButtonBriterEncoderSetCounterClockWiseDirection);
  ButtonRefresh(&hButtonBriterEncoderSetCounterClockWiseDirection);
  ButtonScan(&hButtonBriterEncoderSet1MHzCanRate);
  ButtonRefresh(&hButtonBriterEncoderSet1MHzCanRate);
  ButtonScan(&hButtonBriterEncoderSetCanID);
  ButtonRefresh(&hButtonBriterEncoderSetCanID);
  ButtonScan(&hButtonBriterEncoderSelectEncoder);
  ButtonRefresh(&hButtonBriterEncoderSelectEncoder);
  ButtonScan(&hButtonBriterEncoderIfRead);
  ButtonRefresh(&hButtonBriterEncoderIfRead);
  
  if (ifRequestRead)
    ENCODER_ReadAngleRequest(hEncoderPtr);
  
  LCD_SetLayer(1);
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(100, 10, hEncoderPtr->angleDeg.f, 10, 3);
  LCD_DisplayDecimals(100, 40, hEncoderPtr->speedCalAngleAvgPresent, 10, 3);
  LCD_DisplayDecimals(100, 70, hEncoderPtr->speedDeg.f, 10, 3);
  
  if (ifButtonPressed(&hButtonBriterEncoderZeroing))
  {
    ENCODER_SetZeroPosition(hEncoderPtr);
  }
  else if (ifButtonPressed(&hButtonBriterEncoderSetCounterClockWiseDirection))
  {
//    ENCODER_SetDirection(hEncoderPtr, BRITER_ENCODER_DIRECTION_COUNTERCLOCKWISE);
  }
  else if (ifButtonPressed(&hButtonBriterEncoderSet1MHzCanRate))
  {
    ENCODER_SetAutoFeedbackRate(hEncoderPtr, 5000);
//    ENCODER_Set1MHzCanBaudrate(hEncoderPtr);
  }
  else if (ifButtonPressed(&hButtonBriterEncoderSetCanID))
  {
    ENCODER_Set1MHzCanBaudrate(hEncoderPtr);
  }
  else if (ifButtonPressed(&hButtonBriterEncoderSelectEncoder))
  {
    encoderSelectPtr++;
    if (encoderSelectPtr > 5)
      encoderSelectPtr = 0;
    if (encoderSelectPtr == 0)
    {
      hEncoderPtr = &hEncoderLeftPull;
      LCD_DisplayString(10, 500, "LP");
    }
    else if (encoderSelectPtr == 1)
    {
      hEncoderPtr = &hEncoderLeftTurn;
      LCD_DisplayString(10, 500, "LT");
    }
    else if (encoderSelectPtr == 2)
    {
      hEncoderPtr = &hEncoderRightPull;
      LCD_DisplayString(10, 500, "RP");
    }
    else if (encoderSelectPtr == 3)
    {
      hEncoderPtr = &hEncoderRightTurn;
      LCD_DisplayString(10, 500, "RT");
    }
    else if (encoderSelectPtr == 4)
    {
      hEncoderPtr = &hEncoderLeftWheel;
      LCD_DisplayString(10, 500, "LW");
    }
    else if (encoderSelectPtr == 5)
    {
      hEncoderPtr = &hEncoderRightWheel;
      LCD_DisplayString(10, 500, "RW");
    }
  }
  else if (ifButtonPressed(&hButtonBriterEncoderIfRead))
    ifRequestRead = !ifRequestRead;
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_BriterEncoder_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonBriterEncoderSetCounterClockWiseDirection = Button_Create(100, 200, 300, 40, "Set counterclockwise", LCD_GREEN, LCD_RED);
  hButtonBriterEncoderZeroing = Button_Create(100, 250, 300, 40, "Zeroing", LCD_GREEN, LCD_RED);
  hButtonBriterEncoderSet1MHzCanRate = Button_Create(100, 300, 300, 40, "Set 1MHz", LCD_GREEN, LCD_RED);
  hButtonBriterEncoderSetCanID = Button_Create(100, 350, 300, 40, "Set CAN ID", LCD_GREEN, LCD_RED);
  hButtonBriterEncoderSelectEncoder = Button_Create(10, 600, 300, 40, "Change Encoder", LCD_GREEN, LCD_RED);
  hButtonBriterEncoderIfRead = Button_Create(10, 650, 300, 40, "Read/Not Read", LCD_GREEN, LCD_RED);
}
