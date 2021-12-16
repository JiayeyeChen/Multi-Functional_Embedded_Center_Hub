#include "user_interface.h"
#include "main.h"
#include <math.h>

PageHandle UIPage_Home1, UIPage_AK10_9_Calibration, UIPage_AK10_9_ManualControl;
UIHandle hUI;

ButtonHandle hButtonPageAK10_9Calibration;
ButtonHandle hButtonPageAK10_9ManualControl;
ButtonHandle hButtonDataLogStart;
ButtonHandle hButtonDataLogEnd;
ButtonHandle hButtonMotorProfilingStart;
ButtonHandle hButtonMotorProfilingEnd;
ButtonHandle hButtonMotorZeroing;
ButtonHandle hButtonMotorSteppingUp;
ButtonHandle hButtonMotorSteppingDown;
ButtonHandle hButtonGoBack;

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
  UIPage_AK10_9_Calibration.Page = UI_Page_AK10_9_Calibration;
  UIPage_AK10_9_Calibration.PageInit = UI_Page_AK10_9_Calibration_Init;
  
  UIPage_AK10_9_ManualControl.ifPageInitialized = 0;
  UIPage_AK10_9_ManualControl.Page = UI_Page_AK10_9_ManualControl;
  UIPage_AK10_9_ManualControl.PageInit = UI_Page_AK10_9_ManualControl_Init;
  
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

void UI_Page_AK10_9_Calibration(void)
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

void UI_Page_AK10_9_Calibration_Init(void)
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
  ButtonScan(&hButtonPageAK10_9Calibration);
  ButtonRefresh(&hButtonPageAK10_9Calibration);
  ButtonScan(&hButtonPageAK10_9ManualControl);
  ButtonRefresh(&hButtonPageAK10_9ManualControl);
  
  if (ifButtonPressed(&hButtonPageAK10_9Calibration))
    UI_Page_Change_To(&UIPage_AK10_9_Calibration);
  if (ifButtonPressed(&hButtonPageAK10_9ManualControl))
    UI_Page_Change_To(&UIPage_AK10_9_ManualControl);
  
}
void UI_Page_Home1_Init(void)
{
  hButtonPageAK10_9Calibration = Button_Create(100, 100, 300, 50, "AK10-9 V2.0 Calibration", LIGHT_MAGENTA, LCD_RED);
  hButtonPageAK10_9ManualControl = Button_Create(80, 200, 360, 50, "AK10-9 V2.0 Manual Control", LIGHT_MAGENTA, LCD_RED);
}

void UI_Page_AK10_9_ManualControl(void)
{
  ButtonScan(&hButtonGoBack);
  ButtonScan(&hButtonMotorSteppingUp);
  ButtonScan(&hButtonMotorSteppingDown);
  ButtonScan(&hButtonMotorZeroing);
  ButtonRefresh(&hButtonGoBack);
  ButtonRefresh(&hButtonMotorSteppingUp);
  ButtonRefresh(&hButtonMotorSteppingDown);
  ButtonRefresh(&hButtonMotorZeroing);
  
  if(ifButtonPressed(&hButtonMotorZeroing))
    AK10_9_ServoMode_Zeroing(&hAKMotorLeftHip);
  if(ifButtonPressed(&hButtonMotorSteppingUp))
  {
    positionControlManual+=15.0f;
    AK10_9_ServoMode_PositionControl(&hAKMotorLeftHip, positionControlManual);
  }
  if(ifButtonPressed(&hButtonMotorSteppingDown))
  {
    positionControlManual-=15.0f;
    AK10_9_ServoMode_PositionControl(&hAKMotorLeftHip, positionControlManual);
  }
  
  
  LCD_SetLayer(1); 
  LCD_SetColor(LCD_BLACK);
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
void UI_Page_AK10_9_ManualControl_Init(void)
{
  hButtonGoBack = Button_Create(0, 0, 60, 40, "Back", LCD_WHITE, LCD_RED);
  hButtonMotorSteppingUp = Button_Create(300, 50, 120, 50, "Step up", LCD_WHITE, LCD_RED);
  hButtonMotorSteppingDown = Button_Create(300, 150, 120, 50, "Step down", LCD_WHITE, LCD_RED);
  hButtonMotorZeroing = Button_Create(50, 450, 200, 50, "Motor Set Zero", LCD_BLUE, LCD_RED);
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
