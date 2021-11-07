#include "user_interface.h"


ButtonHandle Button_Create(void)
{
  ButtonHandle hbutton;
  
  return hbutton;
}

void UI_Init(void)
{
  LTDC_Init();
  Touch_Init();
  LCD_DisplayDirection(Direction_V);
}
