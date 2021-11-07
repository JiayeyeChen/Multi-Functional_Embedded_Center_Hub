#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "touch_800x480.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"

#define VIRTUAL_COMPONENT_SHAPE_RECTANGLE 0
#define VIRTUAL_COMPONENT_SHAPE_CIRCLE    1


typedef struct
{
  uint16_t x;
  uint16_t y;
  uint16_t xLen;
  uint16_t yLen;
  uint16_t detectZoneX;
  uint16_t detectZoneY;
  uint16_t detectZoneL;
}VirtualComponentPosition;

typedef struct
{
  VirtualComponentPosition pos;
  
}MenuPageHandle;

typedef struct
{
  VirtualComponentPosition pos;
  uint32_t                 colorPressed;
  uint32_t                 colorUnpressed;
  uint8_t                  ifNeedRefresh;
  uint8_t                  ifPressed;
  uint8_t                  preIfPressed;
}ButtonHandle;

typedef struct
{
  VirtualComponentPosition pos;
}JoystickHandle;



void UI_Init(void);
ButtonHandle Button_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, uint8_t shape, char label[], uint32_t colorUnpressed, uint32_t colorPressed);
uint8_t      ButtonScan(ButtonHandle* hbutton);
uint8_t      ifButtonPressed(ButtonHandle* hbutton);





#endif
