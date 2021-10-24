#ifndef USB_H
#define USB_H

#include "system_periphrals.h"

typedef struct
{
  USBD_HandleTypeDef*   husbd;
  uint8_t*              buf;
  uint32_t*             len;
  char                  rxMsgRaw[50];
}USBHandle;

void USB_Init(void);
void USB_ReceiveCpltCallback(void);


extern USBHandle hUSB;
#endif
