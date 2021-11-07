#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include "main.h"
#include "touch_800x480.h"
#include "lcd_rgb.h"
#include "lcd_pwm.h"

typedef struct
{
  uint8_t test;
}MenuPageHandle;

typedef struct
{
  uint8_t test;
}ButtonHandle;

typedef struct
{
  uint8_t test;
}JoystickHandle;



void UI_Init(void);
ButtonHandle Button_Create(void);






#endif
