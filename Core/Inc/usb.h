#ifndef USB_H
#define USB_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"

typedef struct
{
  USBD_HandleTypeDef*   husbd;
  /*Tx Message*/
  uint8_t              txBuf[256];
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
  /*Data log*/
  uint8_t               dataLogBytes;
  uint8_t               ifNewDataLogPiece2Send;
  uint8_t               ifDataLogInitiated;
  uint8_t               ifDataLogStarted;
  union UInt32UInt8     index;
  
}USBHandle;

void USB_Init(void);
void USB_Transmit_Cargo(uint8_t* buf, uint8_t size);
void USB_ReceiveCpltCallback(void);
void USB_Receive_Cargo(void);
void USB_DataLogInitialization(void);
void USB_DataLogStart(void);
void USB_DataLogEnd(void);
void AK10_9_DataLog_CargoTransmit(AK10_9Handle* hmotor);
void AK10_9_DataLog_Manager(AK10_9Handle* hmotor);

extern USBHandle hUSB;
#endif
