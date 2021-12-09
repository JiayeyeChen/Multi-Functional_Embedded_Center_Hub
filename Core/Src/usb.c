#include "usb.h"

USBHandle hUSB;

void USB_Init(void)
{
  hUSB.husbd = &hUsbDeviceHS;
}

void USB_Transmit_Cargo(uint8_t* buf, uint8_t size)
{
  uint8_t tx_buf[size + 8];
  tx_buf[0] = 0xAA;
  tx_buf[1] = 0xCC;
  tx_buf[2] = size;
  memcpy(&tx_buf[3], buf, size);
  
  union UInt32UInt8 crc;
  uint32_t buf32bit[size + 1];//Include the byte number one
  for (uint8_t i = 0; i <= size; i++)
  {
    buf32bit[i] = (uint32_t)tx_buf[i + 2];
  }
  crc.b32 = HAL_CRC_Calculate(&hcrc, buf32bit, size + 1);
  tx_buf[size + 3] = crc.b8[0];
  tx_buf[size + 4] = crc.b8[1];
  tx_buf[size + 5] = crc.b8[2];
  tx_buf[size + 6] = crc.b8[3];
  tx_buf[size + 7] = 0x55;
  CDC_Transmit_HS(tx_buf, sizeof(tx_buf));
}

void USB_ReceiveCpltCallback(void)
{
  HAL_GPIO_TogglePin(ONBOARD_LED_BLUE_GPIO_Port, ONBOARD_LED_BLUE_Pin);
  HAL_GPIO_TogglePin(ONBOARD_LED_YELLOWGREEN_GPIO_Port, ONBOARD_LED_YELLOWGREEN_Pin);
  memset(hUSB.rxMsgRaw, 0, sizeof(hUSB.rxMsgRaw));
  memcpy(hUSB.rxMsgRaw, hUSB.buf, *hUSB.len);
}
