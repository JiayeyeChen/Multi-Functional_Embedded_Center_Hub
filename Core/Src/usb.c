#include "usb.h"

USBHandle hUSB;

void USB_Init(void)
{
  hUSB.husbd = &hUsbDeviceHS;
}

void USB_ReceiveCpltCallback(void)
{
  HAL_GPIO_TogglePin(ONBOARD_LED_BLUE_GPIO_Port, ONBOARD_LED_BLUE_Pin);
  HAL_GPIO_TogglePin(ONBOARD_LED_YELLOWGREEN_GPIO_Port, ONBOARD_LED_YELLOWGREEN_Pin);
  memset(hUSB.rxMsgRaw, 0, sizeof(hUSB.rxMsgRaw));
  memcpy(hUSB.rxMsgRaw, hUSB.buf, *hUSB.len);
//  for (uint32_t i = 0; i < *hUSB.len; i++)
//  {
//    hUSB.rxMsgRaw[0] = *(hUSB.buf + i);
//  }
  
  
  
  
//  if (*hUSB.buf == 0xA7)
//  {
//    HAL_GPIO_WritePin(ONBOARD_LED_BLUE_GPIO_Port, ONBOARD_LED_BLUE_Pin, GPIO_PIN_RESET);
//    HAL_GPIO_WritePin(ONBOARD_LED_YELLOWGREEN_GPIO_Port, ONBOARD_LED_YELLOWGREEN_Pin, GPIO_PIN_RESET);
//  }
//  else
//  {
//    HAL_GPIO_WritePin(ONBOARD_LED_BLUE_GPIO_Port, ONBOARD_LED_BLUE_Pin, GPIO_PIN_SET);
//    HAL_GPIO_WritePin(ONBOARD_LED_YELLOWGREEN_GPIO_Port, ONBOARD_LED_YELLOWGREEN_Pin, GPIO_PIN_SET);
//  }
}
