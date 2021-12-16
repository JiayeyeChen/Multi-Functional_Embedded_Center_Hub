#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "touch_800x480.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"
#include "usb.h"
#include "tmotor_ak10-9_v2.h"
#include "ak10-9_v2_testing.h"

typedef struct
{
  uint8_t ifPageInitialized;
  void (*Page) (void);
  void (*PageInit) (void);
}PageHandle;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t xLen;
  uint16_t yLen;
}VirtualButtonPosition;

typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t r;
}VirtualJoystickPosition;

typedef struct
{
  VirtualButtonPosition pos;
  uint32_t                 colorPressed;
  uint32_t                 colorUnpressed;
  uint8_t                  ifNeedRefresh;
  uint8_t                  ifPressed;
  uint8_t                  preIfPressed;
}ButtonHandle;

typedef struct
{
  VirtualJoystickPosition pos;
  
}JoystickHandle;

typedef struct
{
  PageHandle* curPage;
  PageHandle* prePage;
}UIHandle;

void UI_Init(void);
ButtonHandle Button_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, char label[], uint32_t colorUnpressed, uint32_t colorPressed);
uint8_t      ButtonScan(ButtonHandle* hbutton);
uint8_t      ifButtonPressed(ButtonHandle* hbutton);
void         ButtonRefresh(ButtonHandle* hbutton);
void         UI(void);
void         UI_Page_Change_To(PageHandle* hpage);
void         UI_Page_AK10_9_Calibration(void);
void         UI_Page_AK10_9_Calibration_Init(void);
void         UI_Page_Home1(void);
void         UI_Page_Home1_Init(void);
void         UI_Page_AK10_9_ManualControl(void);
void         UI_Page_AK10_9_ManualControl_Init(void);

JoystickHandle Joystick_Create(uint16_t x, uint16_t y, uint16_t r, char label[]);


//extern ButtonHandle hButtonDataLogStart, hButtonDataLogEnd, hButtonMotorProfilingStart, hButtonMotorProfilingEnd, hButtonMotorZeroing, hButtonMotorSteppingUp, hButtonMotorSteppingDown;
#endif
