#include "led_control.h"

void LED_Toggle(LEDHandle* hled)
{
  HAL_GPIO_TogglePin(hled->gpioPort, hled->gpioPin);
  if (hled->status == LED_STATUS_OFF)
    hled->status = LED_STATUS_ON;
  else if (hled->status == LED_STATUS_ON)
    hled->status = LED_STATUS_OFF;
}

void LED_Off(LEDHandle* hled)
{
  HAL_GPIO_WritePin(hled->gpioPort, hled->gpioPin, GPIO_PIN_RESET);
  hled->status = LED_STATUS_OFF;
}

void LED_On(LEDHandle* hled)
{
  HAL_GPIO_WritePin(hled->gpioPort, hled->gpioPin, GPIO_PIN_SET);
  hled->status = LED_STATUS_ON;
}

void LED_Blink(LEDHandle* hled, uint32_t freq)
{
  LIMIT_MIN_MAX(freq,0,20);
  if (HAL_GetTick() - hled->lastBlinkTime > 500/freq)
  {
    HAL_GPIO_TogglePin(hled->gpioPort, hled->gpioPin);
    hled->lastBlinkTime = HAL_GetTick();
  }
  hled->blinkFrequency = freq;
  hled->status = LED_STATUS_BLINK;
}
