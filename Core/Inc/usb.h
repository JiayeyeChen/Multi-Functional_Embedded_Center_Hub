#ifndef USB_H
#define USB_H

#include "system_periphrals.h"

typedef struct
{
  USBD_HandleTypeDef*   husbd;
  /*Rx Message*/
  uint8_t*              buf;
  uint32_t*             len;
  char                  rxMsgRaw[264];
  uint8_t               msgDetectStage;
  uint8_t               bytesToRead;
  uint8_t               rxMessageCfrm[256];
  uint8_t               rxMessageLen;
  uint32_t              invalidRxMsgCount;
  uint8_t               ifNewCargo;
}USBHandle;

void USB_Init(void);
void USB_Transmit_Cargo(uint8_t* buf, uint8_t size);
void USB_ReceiveCpltCallback(void);
void USB_Receive_Cargo(void);

extern USBHandle hUSB;
#endif
