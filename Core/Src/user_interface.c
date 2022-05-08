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
ButtonHandle hButtonStepLengthPlus10, hButtonStepLengthMinus10, hButtonStepLengthPlus1, hButtonStepLengthMinus1;
ButtonHandle hButtonManualControlMode;
ButtonHandle hButtonSpringConstantUp, hButtonSpringConstantDown, hButtonDampingConstantUp, hButtonDampingConstantDown;
ButtonHandle hButtonIMUSetModeNDOF, hButtonIMUSetModeGYROONLY;
ButtonHandle hButtonMotorSelectRightHip, hButtonMotorSelectRightKnee;


LinearPotentialmeterHandle  hPotentialmeter;

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

LinearPotentialmeterHandle  Potentialmeter_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, uint16_t sliderLen, uint32_t colorUnpressed, uint32_t colorPressed)
{
  LinearPotentialmeterHandle hpotentialmeter;
  hpotentialmeter.pos.x = x;
  hpotentialmeter.pos.y = y;
  hpotentialmeter.pos.xLen = xLen;
  hpotentialmeter.pos.yLen = yLen;
  hpotentialmeter.pos.xSliderLen = sliderLen;
  
  
  return hpotentialmeter;
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
  
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonDataLogEnd);
  ButtonRefresh(&hButtonDataLogStart);
  ButtonRefresh(&hButtonMotorProfilingStart);
  ButtonRefresh(&hButtonMotorProfilingEnd);
  ButtonRefresh(&hButtonMotorZeroing);
  
  
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
  ButtonScan(&hButtonStepLengthPlus10);
  ButtonScan(&hButtonStepLengthMinus10);
  ButtonScan(&hButtonStepLengthPlus1);
  ButtonScan(&hButtonStepLengthMinus1);
  ButtonScan(&hButtonManualControlMode);
  ButtonScan(&hButtonMotorSelectRightHip);
  ButtonScan(&hButtonMotorSelectRightKnee);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonMotorStart);
  ButtonRefresh(&hButtonMotorStop);
  ButtonRefresh(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonStepLengthPlus10);
  ButtonRefresh(&hButtonStepLengthMinus10);
  ButtonRefresh(&hButtonStepLengthPlus1);
  ButtonRefresh(&hButtonStepLengthMinus1);
  ButtonRefresh(&hButtonManualControlMode);
  ButtonRefresh(&hButtonMotorSelectRightHip);
  ButtonRefresh(&hButtonMotorSelectRightKnee);
  
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_ServoMode_Zeroing(hMotorPtrManualControl);
  if(ifButtonPressed(&hButtonMotorStart))
    ifManualControlStarted = 1;
  if(ifButtonPressed(&hButtonMotorStop))
    ifManualControlStarted = 0;
  if(ifButtonPressed(&hButtonStepLengthPlus10))
    manualControlValue+=10.0f;
  if(ifButtonPressed(&hButtonStepLengthMinus10))
    manualControlValue-=10.0f;
  if(ifButtonPressed(&hButtonStepLengthPlus1))
    manualControlValue+=1.0f;
  if(ifButtonPressed(&hButtonStepLengthMinus1))
    manualControlValue-=1.0f;
  if(ifButtonPressed(&hButtonManualControlMode))
  {
    controlMode++;
    if (controlMode > 3)
      controlMode = 0;
  }
  if (ifButtonPressed(&hButtonMotorSelectRightHip))
    hMotorPtrManualControl = &hAKMotorRightHip;
  else if (ifButtonPressed(&hButtonMotorSelectRightKnee))
    hMotorPtrManualControl = &hAKMotorRightKnee;
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  if (controlMode == AK10_9_MODE_CURRENT)
    LCD_DisplayString(0, 240, "Current  Control");
  else if (controlMode == AK10_9_MODE_POSITION)
    LCD_DisplayString(0, 240, "Position Control");
  else if (controlMode == AK10_9_MODE_VELOCITY)
    LCD_DisplayString(0, 240, "Velocity Control");
  else if (controlMode == AK10_9_MODE_BRAKE)
    LCD_DisplayString(0, 240, "     BRAKE      ");
  LCD_DisplayString(400, 50, "Step: ");
  LCD_DisplayNumber(400, 80, (int32_t)manualControlValue, 3);
  if (hMotorPtrManualControl->status == AK10_9_Online)
    LCD_DisplayString(200, 0, "Motor  Online");
  else
    LCD_DisplayString(200, 0, "Motor Offline");
  LCD_DisplayString(300, 500, "Torque(Nm):");
  LCD_DisplayDecimals(320, 530, (double)hMotorPtrManualControl->realTorque.f, 3, 2);
  LCD_DisplayString(20, 570, "Temperature:");
  LCD_DisplayNumber(70, 600, hMotorPtrManualControl->temperature, 2);
  LCD_DisplayString(230, 600, "Real");
  LCD_DisplayString(360, 600, "Desired");
  LCD_DisplayString(50, 650, "Position: ");LCD_DisplayDecimals(170, 650, (double)hMotorPtrManualControl->realPosition.f, 10, 4);
  LCD_DisplayString(50, 700, "Velocity: ");LCD_DisplayDecimals(170, 700, (double)hMotorPtrManualControl->realVelocity.f, 10, 4);
  LCD_DisplayString(50, 750, "Current:  ");LCD_DisplayDecimals(170, 750, (double)hMotorPtrManualControl->realCurrent.f, 10, 4);
  
  LCD_DisplayDecimals(310, 650, (double)hMotorPtrManualControl->setPosition.f, 10, 4);
  LCD_DisplayDecimals(310, 700, (double)hMotorPtrManualControl->setVelocity.f, 10, 4);
  LCD_DisplayDecimals(310, 750, (double)hMotorPtrManualControl->setCurrent.f, 10, 4);
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_AK10_9_ManualControl_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorStart = Button_Create(0, 50, 100, 50, "START", LCD_WHITE, LCD_RED);
  hButtonMotorStop = Button_Create(0, 120, 100, 50, "STOP", LCD_WHITE, LCD_RED);
  hButtonMotorZeroing = Button_Create(50, 500, 200, 50, "Motor Set Zero", LCD_BLUE, LCD_RED);
  hButtonStepLengthPlus10 = Button_Create(180, 50, 190, 50, "Step Length +10", LCD_WHITE, LCD_RED);
  hButtonStepLengthMinus10 = Button_Create(180, 120, 190, 50, "Step Length -10", LCD_WHITE, LCD_RED);
  hButtonStepLengthPlus1 = Button_Create(210, 180, 190, 50, "Step Length +1", LCD_WHITE, LCD_RED);
  hButtonStepLengthMinus1 = Button_Create(210, 250, 190, 50, "Step Length -1", LCD_WHITE, LCD_RED);
  hButtonManualControlMode = Button_Create(0, 180, 160, 50, "Control Mode", LCD_WHITE, LCD_RED);
  hButtonMotorSelectRightHip = Button_Create(0, 300, 160, 50, "Right Hip", DARK_YELLOW, LCD_RED);
  hButtonMotorSelectRightKnee = Button_Create(0, 400, 160, 50, "Right Knee", DARK_YELLOW, LCD_RED);
  
  hMotorPtrManualControl = &hAKMotorRightHip;
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
  ButtonScan(&hButtonIMUSetModeGYROONLY);
  ButtonRefresh(&hButtonIMUSetModeGYROONLY);
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
  LCD_DisplayDecimals(200, 0, (double)hIMURightThigh.rawData.liaccX.b16, 7, 1);
  LCD_DisplayDecimals(200, 50, (double)hIMURightThigh.rawData.liaccY.b16, 7, 1);
  LCD_DisplayDecimals(200, 100, (double)hIMURightThigh.rawData.liaccZ.b16, 7, 1);
  LCD_DisplayDecimals(200, 150, (double)hIMURightThigh.rawData.gyroX.b16, 7, 1);
  LCD_DisplayDecimals(200, 200, (double)hIMURightThigh.rawData.gyroY.b16, 7, 1);
  LCD_DisplayDecimals(200, 250, (double)hIMURightThigh.rawData.gyroZ.b16, 7, 1);
  
  if (hIMURightThigh.operationModeENUM == IMU_MODE_NDOF)
    EXOSKELETON_SetIMUMode_9_DOF(&hIMURightThigh);
  else if (hIMURightThigh.operationModeENUM == IMU_MODE_GYROONLY)
    EXOSKELETON_SetIMUMode_GYRO_Only(&hIMURightThigh);
  
  if (ifButtonPressed(&hButtonIMUSetModeNDOF))
    hIMURightThigh.operationModeENUM = IMU_MODE_NDOF;
  if (ifButtonPressed(&hButtonIMUSetModeGYROONLY))
    hIMURightThigh.operationModeENUM = IMU_MODE_GYROONLY;
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}

void UI_Page_BNO055_Monitor_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonIMUSetModeNDOF = Button_Create(100, 300, 250, 40, "9 DOF Fusion Mode", LIGHT_YELLOW, LCD_RED);
  hButtonIMUSetModeGYROONLY = Button_Create(100, 350, 200, 40, "Gyro Only Mode", LIGHT_YELLOW, LCD_RED);
  LCD_DisplayString(100, 0, "LiAccX: ");
  LCD_DisplayString(100, 50, "LiAccY: ");
  LCD_DisplayString(100, 100, "LiAccZ: ");
  LCD_DisplayString(100, 150, "GyroX:  ");
  LCD_DisplayString(100, 200, "GyroY:  ");
  LCD_DisplayString(100, 250, "GyroZ:  ");
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
  
  if (ifButtonPressed(&hButtonGoBack))
    UI_Page_Change_To(&UIPage_Home1);
}
void UI_Page_TMotor_Acceleration_Observer_Project_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
}
