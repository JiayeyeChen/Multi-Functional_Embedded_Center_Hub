#include "user_interface.h"
#include "main.h"
#include <math.h>

ButtonHandle Button_Create(uint16_t x, uint16_t y, uint16_t xLen, uint16_t yLen, uint8_t shape, char label[],\
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
  hbutton.shape = shape;


  if (shape == VIRTUAL_COMPONENT_SHAPE_RECTANGLE)
  {
    LCD_SetLayer(0);
    LCD_SetColor(hbutton.colorUnpressed);
    LCD_FillRect(hbutton.pos.x, hbutton.pos.y, hbutton.pos.xLen, hbutton.pos.yLen);
    LCD_SetLayer(1);
    LCD_SetColor(LCD_BLACK);
    LCD_DrawRect(hbutton.pos.x, hbutton.pos.y, hbutton.pos.xLen, hbutton.pos.yLen);
    LCD_DisplayString(hbutton.pos.x + 10, hbutton.pos.y + 5, (char*)label);
    hbutton.pos.detectZoneX = hbutton.pos.x + hbutton.pos.xLen / 2;
    hbutton.pos.detectZoneY = hbutton.pos.y + hbutton.pos.yLen / 2;
    hbutton.pos.detectZoneL = MIN(hbutton.pos.xLen, hbutton.pos.yLen) / 2;
  }
  else if (shape == VIRTUAL_COMPONENT_SHAPE_CIRCLE)
  {
    LCD_SetLayer(0);
    LCD_SetColor(hbutton.colorUnpressed);
    LCD_FillCircle(hbutton.pos.x + hbutton.pos.xLen / 2, hbutton.pos.y + hbutton.pos.yLen / 2, MIN(hbutton.pos.yLen, hbutton.pos.xLen)/2);
    LCD_SetLayer(1);
    LCD_SetColor(LCD_BLACK);
    LCD_DrawCircle(hbutton.pos.x + hbutton.pos.xLen / 2, hbutton.pos.y + hbutton.pos.yLen / 2, MIN(hbutton.pos.yLen, hbutton.pos.xLen)/2);
    LCD_DisplayString(hbutton.pos.x + 10, hbutton.pos.y + hbutton.pos.yLen / 2 - 10, (char*)label);
    hbutton.pos.detectZoneX = hbutton.pos.x + hbutton.pos.xLen / 2;
    hbutton.pos.detectZoneY = hbutton.pos.y + hbutton.pos.yLen / 2;
    hbutton.pos.detectZoneL = (uint16_t)((sqrt(2.0f) / 2.0f) * (float)MIN(hbutton.pos.yLen, hbutton.pos.xLen)/2.0f);
  }
  return hbutton;
}

uint8_t ButtonScan(ButtonHandle* hbutton)
{
  for (uint8_t i = 0; i <= 4; i++)
  {
    if ((touchInfo.xVertical[i] > (hbutton->pos.detectZoneX - hbutton->pos.detectZoneL) && touchInfo.xVertical[i] < (hbutton->pos.detectZoneX + hbutton->pos.detectZoneL)) &&\
        (touchInfo.yVertical[i] > (hbutton->pos.detectZoneY - hbutton->pos.detectZoneL) && touchInfo.yVertical[i] < (hbutton->pos.detectZoneY + hbutton->pos.detectZoneL)))
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
}

JoystickHandle Joystick_Create(uint16_t x, uint16_t y, uint16_t r, char label[])
{
  JoystickHandle hjoy;
  hjoy.pos.x = x;
  hjoy.pos.y = y;
  hjoy.pos.xLen = r;
  hjoy.pos.yLen = r;
  LCD_SetLayer(0);
  LCD_SetColor(DARK_CYAN);
  LCD_FillCircle(hjoy.pos.x + hjoy.pos.xLen, hjoy.pos.y + hjoy.pos.yLen, hjoy.pos.xLen);
  LCD_SetColor(LCD_BLACK);
  LCD_DrawCircle(hjoy.pos.x + hjoy.pos.xLen, hjoy.pos.y + hjoy.pos.yLen, hjoy.pos.xLen);
  LCD_DisplayString(hjoy.pos.x, hjoy.pos.y - 30, (char*)label);
  
//  hjoy.pos.detectZoneX = 
//  hjoy.pos.detectZoneY = 
//  hjoy.pos.detectZoneL = 
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
    if (hbutton->shape == VIRTUAL_COMPONENT_SHAPE_RECTANGLE)
      LCD_FillRect(hbutton->pos.x, hbutton->pos.y, hbutton->pos.xLen, hbutton->pos.yLen);
    else if (hbutton->shape == VIRTUAL_COMPONENT_SHAPE_CIRCLE)
      LCD_FillCircle(hbutton->pos.x + hbutton->pos.xLen / 2, hbutton->pos.y + hbutton->pos.yLen / 2, MIN(hbutton->pos.yLen, hbutton->pos.xLen)/2);
    hbutton->ifNeedRefresh = 0;
  }
}
