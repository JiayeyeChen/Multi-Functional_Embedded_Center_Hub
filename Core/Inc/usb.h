#ifndef USB_H
#define USB_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"
#include <string.h>
#include "cmsis_os2.h"
#include "common.h"
#include <stdarg.h>

enum DatalogType
{
  DATALOG_TYPE_NOLIMIT,
  DATALOG_TYPE_TIMESEGMENT
};

enum DataLogTask
{
  DATALOG_TASK_FREE,
  DATALOG_TASK_START,
  DATALOG_TASK_SEND_DATA_SLOT_LEN,
  DATALOG_TASK_SEND_DATA_SLOT_MSG,
  DATALOG_TASK_DATALOG,
  DATALOG_TASK_END
};

typedef struct
{
  USBD_HandleTypeDef*   husbd;
  /*Tx Message*/
  uint8_t               txBuf[256];
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
  uint8_t               ifNewDataLogPiece2Send;
  uint8_t               ifDataLogInitialized;
  union UInt32UInt8     index;
  uint8_t               dataSlotLen;
  uint32_t              datalogStartTimestamp;
  uint32_t              timeSegmentDuration;
  enum DataLogTask      datalogTask;
}USBHandle;

void USB_Init(uint8_t data_slot_len);
void USB_SetNewDataSlotLen(uint8_t data_slot_len);
void USB_TransmitCargo(uint8_t* buf, uint8_t size);
void USB_SendText(char text[]);
void USB_ReceiveCpltCallback(void);
void USB_ReceiveCargo(void);
void USB_DatalogCargoReceiveManager(void (*LabelSetFunc)(void));
void USB_DataLogManager(void (*LabelSetFunc)(void), union FloatUInt8 dala_slots[]);
void USB_DataLogInitialization(void);
void USB_DataLogStart(void);
void USB_DataLogEnd(void);
void USB_SendDataSlotLen(void);
void USB_SendDataSlotLabel(char* label_1, ...);
void USB_DataLogSingleCargoTransmit(union FloatUInt8 dala_slots[]);
uint8_t USB_CompareRxCfmMsgWithStr(char str[], uint8_t size_of_str);

void USB_DataLogConfigureDataSlot(float* data, uint8_t len);

extern USBHandle hUSB;
extern union FloatUInt8 dataSlots_AK10_9_Acceleration_Observer_Testing[11];
extern union FloatUInt8 dataSlots_AK10_9_TorqueConstantTesting[2];
extern union FloatUInt8 dataSlots_Exoskeleton_SystemID[9];
#endif
