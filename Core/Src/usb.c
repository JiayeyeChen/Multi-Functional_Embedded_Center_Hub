#include "usb.h"

USBHandle hUSB;
union FloatUInt8 dataSlots_AK10_9_Acceleration_Observer_Testing[11];
union FloatUInt8 dataSlots_AK10_9_TorqueConstantTesting[2];

void USB_Init(uint8_t data_slot_len)
{
  hUSB.husbd = &hUsbDeviceHS;
  hUSB.invalidRxMsgCount = 0;
  hUSB.ifNewCargo = 0;
  hUSB.ifNewDataLogPiece2Send = 0;
  hUSB.index.b32 = 0;
  hUSB.datalogTask = DATALOG_TASK_FREE;
  hUSB.dataSlotLen = data_slot_len;
}

void USB_SendText(char text[])
{
  USB_TransmitCargo((uint8_t*)text, strlen(text));
}

void USB_TransmitCargo(uint8_t* buf, uint8_t size)
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
  memset(hUSB.rxMsgRaw, 0, sizeof(hUSB.rxMsgRaw));
  memcpy(hUSB.rxMsgRaw, hUSB.buf, *hUSB.len);
  USB_ReceiveCargo();
}

void USB_ReceiveCargo(void)
{
  if (hUSB.rxMsgRaw[0] == (char)0xBB && hUSB.rxMsgRaw[1] == (char)0xCC && hUSB.rxMsgRaw[*hUSB.len - 1] == (char)0x88)
  {
    if (hUSB.rxMsgRaw[2] == *hUSB.len - 8)
    {
      uint32_t crcCalculatedResult;
      union UInt32UInt8 crcReceive;
      
      hUSB.rxMessageLen = hUSB.rxMsgRaw[2];
      uint32_t crcCalculate[hUSB.rxMessageLen + 1];
      for (uint8_t i = 0; i<= hUSB.rxMessageLen; i++)
        crcCalculate[i] = (uint32_t)hUSB.rxMsgRaw[2 + i];
      crcCalculatedResult = HAL_CRC_Calculate(&hcrc, crcCalculate, hUSB.rxMessageLen + 1);
      
      crcReceive.b8[0] = hUSB.rxMsgRaw[*hUSB.len - 5];
      crcReceive.b8[1] = hUSB.rxMsgRaw[*hUSB.len - 4];
      crcReceive.b8[2] = hUSB.rxMsgRaw[*hUSB.len - 3];
      crcReceive.b8[3] = hUSB.rxMsgRaw[*hUSB.len - 2];
      
//      memcpy(hUSB.rxMessageCfrm, &hUSB.rxMsgRaw[3], hUSB.rxMessageLen);
//      hUSB.ifNewCargo = 1;
      if (crcCalculatedResult == crcReceive.b32)
      {
        memcpy(hUSB.rxMessageCfrm, &hUSB.rxMsgRaw[3], hUSB.rxMessageLen);
        hUSB.ifNewCargo = 1;
      }
    }
    else
    {
      hUSB.invalidRxMsgCount++;
      return;
    }
  }
  else
  {
    hUSB.invalidRxMsgCount++;
    return;
  }
}

void USB_CargoReceiveManager(void (*LabelSetFunc)(void))
{
  if (hUSB.ifNewCargo)
  {
    char msg[hUSB.rxMessageLen];
    memcpy(msg, hUSB.rxMessageCfrm, hUSB.rxMessageLen);
    if (hUSB.datalogTask == DATALOG_TASK_FREE)
    {
      if (!strcmp(msg, "Datalog start"))
      {
        hUSB.datalogTask = DATALOG_TASK_SEND_DATA_SLOT_LEN;
        USB_SendDataSlotLen();
      }
    }
    else if (hUSB.datalogTask == DATALOG_TASK_START)
    {
      if (!strcmp(msg, "Roger that"))
      {
        hUSB.datalogTask = DATALOG_TASK_SEND_DATA_SLOT_LEN;
        USB_SendDataSlotLen();
      }
    }
    else if (hUSB.datalogTask == DATALOG_TASK_SEND_DATA_SLOT_LEN)
    {
      if (!strcmp(msg, "Roger that"))
      {
        hUSB.datalogTask = DATALOG_TASK_SEND_DATA_SLOT_MSG;
        (*LabelSetFunc)();
      }
    }
    else if (hUSB.datalogTask == DATALOG_TASK_SEND_DATA_SLOT_MSG)
    {
      if (!strcmp(msg, "Roger that"))
      {
        hUSB.datalogTask = DATALOG_TASK_DATALOG;
      }
    }
    else if (hUSB.datalogTask == DATALOG_TASK_DATALOG)
    {
      if (!strcmp(msg, "Datalog end"))
        hUSB.datalogTask = DATALOG_TASK_FREE;
      USB_SendText("Roger that");
    }
    else if (hUSB.datalogTask == DATALOG_TASK_END)
    {
      if (!strcmp(msg, "Roger that"))
        hUSB.datalogTask = DATALOG_TASK_FREE;
    }
    
    hUSB.ifNewCargo = 0;
  }
}

void USB_DataLogManager(void (*LabelSetFunc)(void), union FloatUInt8 dala_slots[])
{
  USB_CargoReceiveManager(LabelSetFunc);
  if (hUSB.datalogTask == DATALOG_TASK_DATALOG)
  {
    if (hUSB.ifNewDataLogPiece2Send)
    {
      USB_DataLogSingleCargoTransmit(dala_slots);
      hUSB.ifNewDataLogPiece2Send = 0;
    }
  }
}

void USB_DataLogSingleCargoTransmit(union FloatUInt8 dala_slots[])
{
  uint8_t i = 0;
  union UInt32UInt8 sysTick;
  sysTick.b32 = HAL_GetTick();
  //Index
  hUSB.txBuf[i++] = hUSB.index.b8[0];
  hUSB.txBuf[i++] = hUSB.index.b8[1];
  hUSB.txBuf[i++] = hUSB.index.b8[2];
  hUSB.txBuf[i++] = hUSB.index.b8[3];
  hUSB.index.b32++;
  //Time stamp
  hUSB.txBuf[i++] = sysTick.b8[0];
  hUSB.txBuf[i++] = sysTick.b8[1];
  hUSB.txBuf[i++] = sysTick.b8[2];
  hUSB.txBuf[i++] = sysTick.b8[3];
  //Data
  for (uint8_t j = 0; j < hUSB.dataSlotLen; j++)
  {
    hUSB.txBuf[i++] = dala_slots[j].b8[0];
    hUSB.txBuf[i++] = dala_slots[j].b8[1];
    hUSB.txBuf[i++] = dala_slots[j].b8[2];
    hUSB.txBuf[i++] = dala_slots[j].b8[3];
  }
  USB_TransmitCargo(hUSB.txBuf, hUSB.dataSlotLen * 4 + 8);
}

void USB_DataLogInitialization(void)
{
  hUSB.ifDataLogInitialized = 1;
  hUSB.ifNewDataLogPiece2Send = 0;
  hUSB.index.b32 = 0;
}

void USB_DataLogStart(void)
{
  USB_DataLogInitialization();
  hUSB.datalogTask = DATALOG_TASK_START;
  USB_SendText("Datalog start");
}
void USB_DataLogEnd(void)
{
  hUSB.datalogTask = DATALOG_TASK_END;
  USB_SendText("Datalog end");
}

void USB_SendDataSlotLen(void)
{
  char numStr[2];
  int numStrLen;
  numStrLen = sprintf(numStr, "%d", hUSB.dataSlotLen);
  USB_TransmitCargo((uint8_t*)numStr, numStrLen);
}

void USB_SendDataSlotLabel(char* label_1, ...)
{
  va_list label_ptr;
  va_start(label_ptr, label_1);
 
  uint8_t numOfLabels = atoi(label_1);
  for (uint8_t i = 0; i < numOfLabels; i++)
  {
    char buf[50];
    strcpy(buf, va_arg(label_ptr, char*));
    USB_TransmitCargo((uint8_t*)buf, strlen(buf));
    osDelay(10);
  }
  va_end(label_ptr);
}
