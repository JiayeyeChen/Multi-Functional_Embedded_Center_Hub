#include "user_interface.h"
#include "main.h"
#include <math.h>

PageHandle UIPage_Home1, UIPage_AK10_9_Calibration, UIPage_BNO055_Monitor, \
           UIPage_AK10_9_ManualControl, UIPage_AK10_9_ImpedanceControlDemo, \
           UIPage_TMotor_Acceleration_Observer_Project;
UIHandle hUI;

ButtonHandle hButtonPageAK10_9KtTesting;
ButtonHandle hButtonPageAK10_9ManualControl;
ButtonHandle hButtonPageAK10_9ImpedanceControlDemo;
ButtonHandle hButtonPageBNO055_Monitor;
ButtonHandle hButtonPageTMotorAccelerationObserverProject;

ButtonHandle hButtonDataLogStart;
ButtonHandle hButtonDataLogEnd;
ButtonHandle hButtonMotorProfilingStart;
ButtonHandle hButtonMotorProfilingEnd;
ButtonHandle hButtonMotorZeroing;
ButtonHandle hButtonMotorStart;
ButtonHandle hButtonMotorStop;
ButtonHandle hButtonGoBack;
ButtonHandle hButtonManualControlMode;
ButtonHandle hButtonSpringConstantUp, hButtonSpringConstantDown, hButtonDampingConstantUp, hButtonDampingConstantDown;
ButtonHandle hButtonIMUSetModeNDOF, hButtonIMUSetModeACCONLY, hButtonIMUSetModeGYROONLY;
ButtonHandle hButtonMotorSelectRightHip, hButtonMotorSelectRightKnee;


LinearPotentialmeterHandle  hTMotorManualControlPot_pos, hTMotorManualControlPot_vel, hTMotorManualControlPot_cur;
LinearPotentialmeterHandle  hPotTMotorProfilingFrequency;

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
  
  UIPage_AK10_9_Calibration.ifPageInitialized = 0;
  UIPage_AK10_9_Calibration.Page = UI_Page_AK10_9_Kt_Testing;
  UIPage_AK10_9_Calibration.PageInit = UI_Page_AK10_9_Kt_Testing_Init;
  
  UIPage_AK10_9_ManualControl.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControl.Page = UI_Page_AK10_9_ManualControl;
  UIPage_AK10_9_ManualControl.PageInit = UI_Page_AK10_9_ManualControl_Init;
  
  UIPage_AK10_9_ImpedanceControlDemo.ifPageInitialized = 0;
  UIPage_AK10_9_ImpedanceControlDemo.Page = UI_Page_AK10_9_ImpedanceControlDemo;
  UIPage_AK10_9_ImpedanceControlDemo.PageInit = UI_Page_AK10_9_ImpedanceControlDemo_Init;
  
  UIPage_BNO055_Monitor.ifPageInitialized = 0;
  UIPage_BNO055_Monitor.Page = UI_Page_BNO055_Monitor;
  UIPage_BNO055_Monitor.PageInit = UI_Page_BNO055_Monitor_Init;
  
  UIPage_TMotor_Acceleration_Observer_Project.ifPageInitialized = 0;
  UIPage_TMotor_Acceleration_Observer_Project.Page = UI_Page_TMotor_Acceleration_Observer_Project;
  UIPage_TMotor_Acceleration_Observer_Project.PageInit = UI_Page_TMotor_Acceleration_Observer_Project_Init;
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

void UI_Page_AK10_9_Kt_Testing(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonDataLogStart);
  ButtonScan(&hButtonDataLogEnd);
  ButtonScan(&hButtonMotorProfilingStart);
  ButtonScan(&hButtonMotorProfilingEnd);
  ButtonScan(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonDataLogEnd);
  ButtonRefresh(&hButtonDataLogStart);
  ButtonRefresh(&hButtonMotorProfilingStart);
  ButtonRefresh(&hButtonMotorProfilingEnd);
  ButtonRefresh(&hButtonMotorZeroing);
  
  if (ifButtonPressed(&hButtonDataLogStart))
  {
    USB_DataLogStart();
  }
  if (ifButtonPressed(&hButtonDataLogEnd))
  {
    USB_DataLogEnd();
  }
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
  {
    AK10_9_ServoMode_Zeroing(&hAKMotorLeftHip);
  }
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hAKMotorLeftHip.status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  LCD_DisplayString(20, 570, "Temperature:");
  LCD_DisplayNumber(70, 600, hAKMotorLeftHip.temperature, 2);
  LCD_DisplayString(230, 600, "Real");
  LCD_DisplayString(360, 600, "Desired");
  LCD_DisplayString(50, 650, "Position: ");LCD_DisplayDecimals(170, 650, (double)hAKMotorLeftHip.realPosition.f, 10, 4);
  LCD_DisplayString(50, 700, "Velocity: ");LCD_DisplayDecimals(170, 700, (double)hAKMotorLeftHip.realVelocity.f, 10, 4);
  LCD_DisplayString(50, 750, "Current:  ");LCD_DisplayDecimals(170, 750, (double)hAKMotorLeftHip.realCurrent.f, 10, 4);
  
  LCD_DisplayDecimals(310, 650, (double)hAKMotorLeftHip.setPosition.f, 10, 4);
  LCD_DisplayDecimals(310, 700, (double)hAKMotorLeftHip.setVelocity.f, 10, 4);
  LCD_DisplayDecimals(310, 750, (double)hAKMotorLeftHip.setCurrent.f, 10, 4);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_AK10_9_Kt_Testing_Init(void)
{
  hButtonDataLogStart = Button_Create(50, 50, 200, 50, "Data Log Start", LIGHT_MAGENTA, LCD_RED);
  hButtonDataLogEnd = Button_Create(50, 150, 200, 50, "Data Log End", LCD_GREEN, LCD_RED);
  hButtonMotorProfilingStart = Button_Create(50, 250, 300, 50, "Motor profiling Start", LIGHT_GREY, LCD_RED);
  hButtonMotorProfilingEnd = Button_Create(50, 350, 300, 50, "Motor profiling Stop", LCD_YELLOW, LCD_RED);
  hButtonMotorZeroing = Button_Create(50, 450, 200, 50, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
}

void UI_Page_Home1(void)
{
  LCD_SetFont(&Font32); 
  LCD_DisplayString(140, 30, "Welcome Jiaye");
  LCD_SetFont(&Font24);
  ButtonScan(&hButtonPageAK10_9KtTesting);
  ButtonRefresh(&hButtonPageAK10_9KtTesting);
  ButtonScan(&hButtonPageAK10_9ManualControl);
  ButtonRefresh(&hButtonPageAK10_9ManualControl);
  ButtonScan(&hButtonPageAK10_9ImpedanceControlDemo);
  ButtonRefresh(&hButtonPageAK10_9ImpedanceControlDemo);
  ButtonScan(&hButtonPageBNO055_Monitor);
  ButtonRefresh(&hButtonPageBNO055_Monitor);
  ButtonScan(&hButtonPageTMotorAccelerationObserverProject);
  ButtonRefresh(&hButtonPageTMotorAccelerationObserverProject);
  
  if (ifButtonPressed(&hButtonPageAK10_9KtTesting))
    UI_Page_Change_To(&UIPage_AK10_9_Calibration);
  if (ifButtonPressed(&hButtonPageAK10_9ManualControl))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControl);
  if (ifButtonPressed(&hButtonPageAK10_9ImpedanceControlDemo))
    UI_Page_Change_To(&UIPage_AK10_9_ImpedanceControlDemo);
  if (ifButtonPressed(&hButtonPageBNO055_Monitor))
    UI_Page_Change_To(&UIPage_BNO055_Monitor);
  if (ifButtonPressed(&hButtonPageTMotorAccelerationObserverProject))
    UI_Page_Change_To(&UIPage_TMotor_Acceleration_Observer_Project);
}
void UI_Page_Home1_Init(void)
{
  hButtonPageAK10_9KtTesting = Button_Create(100, 100, 300, 50, "AK10-9 V2.0 Kt Testing", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAK10_9ManualControl = Button_Create(80, 200, 360, 50, "AK10-9 V2.0 Manual Control", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAK10_9ImpedanceControlDemo = Button_Create(30, 300, 430, 50, "AK10-9 V2.0 Impedance Control Demo", LIGHT_MAGENTA, LCD_RED);
  hButtonPageBNO055_Monitor = Button_Create(150, 400, 200, 50, "BNO055 Monitor", LIGHT_MAGENTA, LCD_RED);
  hButtonPageTMotorAccelerationObserverProject = Button_Create(10, 500, 450, 50, "TMotor Acceleration Observer Project", LIGHT_MAGENTA, LCD_RED);
}

void UI_Page_AK10_9_ManualControl(void)
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
    controlMode++;
    if (controlMode > 3)
      controlMode = 0;
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
  if (controlMode == AK10_9_MODE_CURRENT)
  {
    LCD_DisplayString(0, 340, "Current  Control");
    LCD_SetColor(LCD_RED);
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_cur, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlMode == AK10_9_MODE_POSITION)
  {
    LCD_DisplayString(0, 340, "Position Control");
    LCD_SetColor(LCD_RED);
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_pos, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlMode == AK10_9_MODE_VELOCITY)
  {
    LCD_DisplayString(0, 340, "Velocity Control");
    LCD_SetColor(LCD_RED);
    
    LCD_DisplayDecimals(0, 370, (double)manualControlValue_vel, 4, 2);
    LCD_SetColor(LCD_BLACK);
  }
  else if (controlMode == AK10_9_MODE_BRAKE)
    LCD_DisplayString(0, 340, "     BRAKE      ");
  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  LCD_DisplayDecimals(400, 735, (double)hMotorPtrManualControl->realTorque.f, 3, 2);
  LCD_DisplayNumber(400, 760, hMotorPtrManualControl->temperature, 2);
  LCD_DisplayDecimals(150, 710, (double)hMotorPtrManualControl->realPosition.f, 6, 3);
  LCD_DisplayDecimals(150, 735, (double)hMotorPtrManualControl->realVelocity.f, 6, 3);
  LCD_DisplayDecimals(150, 760, (double)hMotorPtrManualControl->realCurrent.f, 6, 3);
  LCD_DisplayDecimals(250, 710, (double)hMotorPtrManualControl->setPosition.f, 6, 3);
  LCD_DisplayDecimals(250, 735, (double)hMotorPtrManualControl->setVelocity.f, 6, 3);
  LCD_DisplayDecimals(250, 760, (double)hMotorPtrManualControl->setCurrent.f, 6, 3);
  
  if (ifButtonPressed(&hButtonGoBack))
  {
    UI_Page_Change_To(&UIPage_Home1);
    ifManualControlStarted = 0;
  }
}
void UI_Page_AK10_9_ManualControl_Init(void)
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

void UI_Page_AK10_9_ImpedanceControlDemo(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonRefresh(&hButtonGoBack);
  ButtonScan(&hButtonSpringConstantUp);
  ButtonRefresh(&hButtonSpringConstantUp);
  ButtonScan(&hButtonSpringConstantDown);
  ButtonRefresh(&hButtonSpringConstantDown);
  ButtonScan(&hButtonDampingConstantUp);
  ButtonRefresh(&hButtonDampingConstantUp);
  ButtonScan(&hButtonDampingConstantDown);
  ButtonRefresh(&hButtonDampingConstantDown);
  ButtonScan(&hButtonMotorStart);
  ButtonRefresh(&hButtonMotorStart);
  ButtonScan(&hButtonMotorStop);
  ButtonRefresh(&hButtonMotorStop);
  ButtonScan(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonMotorZeroing);
  if (ifButtonPressed(&hButtonSpringConstantUp))
    impedance_control_spring_constant += 0.005f;
  if (ifButtonPressed(&hButtonSpringConstantDown))
    impedance_control_spring_constant -= 0.005f;
  if (ifButtonPressed(&hButtonDampingConstantUp))
    impedance_control_damping_constant += 0.001f;
  if (ifButtonPressed(&hButtonDampingConstantDown))
    impedance_control_damping_constant -= 0.001f;
  if (ifButtonPressed(&hButtonMotorStart))
    ifImpedanceControlStarted = 1;
  if (ifButtonPressed(&hButtonMotorStop))
    ifImpedanceControlStarted = 0;
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_ServoMode_Zeroing(&hAKMotorLeftHip);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(280, 100, (double)impedance_control_spring_constant, 10, 4);
  LCD_DisplayDecimals(280, 200, (double)impedance_control_damping_constant, 10, 4);
  if (hAKMotorLeftHip.status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  LCD_DisplayString(300, 500, "Torque(Nm):");
  LCD_DisplayDecimals(320, 530, (double)hAKMotorLeftHip.realTorque.f, 3, 2);
  LCD_DisplayString(20, 570, "Temperature:");
  LCD_DisplayNumber(70, 600, hAKMotorLeftHip.temperature, 2);
  LCD_DisplayString(230, 600, "Real");
  LCD_DisplayString(360, 600, "Desired");
  LCD_DisplayString(50, 650, "Position: ");LCD_DisplayDecimals(170, 650, (double)hAKMotorLeftHip.realPosition.f, 10, 4);
  LCD_DisplayString(50, 700, "Velocity: ");LCD_DisplayDecimals(170, 700, (double)hAKMotorLeftHip.realVelocity.f, 10, 4);
  LCD_DisplayString(50, 750, "Current:  ");LCD_DisplayDecimals(170, 750, (double)hAKMotorLeftHip.realCurrent.f, 10, 4);
  
  LCD_DisplayDecimals(310, 650, (double)hAKMotorLeftHip.setPosition.f, 10, 4);
  LCD_DisplayDecimals(310, 700, (double)hAKMotorLeftHip.setVelocity.f, 10, 4);
  LCD_DisplayDecimals(310, 750, (double)hAKMotorLeftHip.setCurrent.f, 10, 4);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_AK10_9_ImpedanceControlDemo_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonSpringConstantUp = Button_Create(20, 80, 250, 40, "Spring Constant +", LCD_WHITE, LCD_RED);
  hButtonSpringConstantDown = Button_Create(20, 130, 250, 40, "Spring Constant -", LCD_WHITE, LCD_RED);
  hButtonDampingConstantUp = Button_Create(20, 180, 250, 40, "Damping Constant +", LCD_WHITE, LCD_RED);
  hButtonDampingConstantDown = Button_Create(20, 230, 250, 40, "Damping Constant -", LCD_WHITE, LCD_RED);
  hButtonMotorZeroing = Button_Create(50, 500, 200, 50, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonMotorStart = Button_Create(100, 300, 250, 40, "       START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(100, 360, 250, 40, "        STOP", LCD_WHITE, LCD_RED);
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
    AK10_9_ServoMode_Zeroing(&hAKMotorRightHip);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (hAKMotorRightHip.status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  
  
  LCD_DisplayDecimals(140, 720, (double)hAKMotorRightHip.realPosition.f, 10, 4);
  LCD_DisplayDecimals(140, 745, (double)hAKMotorRightHip.setPosition.f, 10, 4);
  LCD_DisplayDecimals(140, 770, (double)hAKMotorRightHip.realVelocity.f, 10, 4);
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
  LCD_DisplayDecimals(230, 445, (double)(hAKMotorRightHip.realPosition.f - hAKMotorRightHip.setPosition.f), 3, 4);
  LCD_SetColor(LCD_BLACK);
  
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
  
  ifIMUFeedbackStarted = 1;
}
