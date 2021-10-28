#ifndef LED_CONTROL_H
#define LED_CONTROL_H

#include "main.h"

enum LEDStatus
{
  LED_STATUS_ON,
  LED_STATUS_OFF,
  LED_STATUS_BLINK
};

typedef struct
{
  GPIO_TypeDef*     gpioPort;
  uint16_t		      gpioPin;
  enum LEDStatus    status;
  uint32_t          blinkFrequency;
  uint32_t          lastBlinkTime;
}LEDHandle;


void LED_Toggle(LEDHandle* hled);

void LED_Off(LEDHandle* hled);

void LED_On(LEDHandle* hled);

void LED_Blink(LEDHandle* hled, uint32_t freq);













#endif
