#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include "common.h"
#include "system_periphrals.h"

typedef struct
{
	union Int16UInt8 liaccX;
	union Int16UInt8 liaccY;
	union Int16UInt8 liaccZ;
	union Int16UInt8 gyroX;
	union Int16UInt8 gyroY;
	union Int16UInt8 gyroZ;
	union Int16UInt8 Quat[4];
}BNO055_Data_Struct;

typedef struct
{
	uint8_t										operationMode;
	uint8_t										calibStatus;
	uint8_t										deadCount;
	BNO055_Data_Struct				data;
  
  CAN_HandleTypeDef*    hcan;
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t*             pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
  //CAN BUS receive
  uint8_t               rxBuf[8];
  uint32_t              rxFifo;
  CAN_FilterTypeDef     rxFilter;
  uint32_t              lastReceivedTime;
}BNO055Handle;

void EXOSKELETON_Init(void);
void EXOSKELETON_GetIMUFeedbackLiAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackGyro(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackQuaternion(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackStatus(BNO055Handle* himu, uint8_t data[]);


extern BNO055Handle hIMURightThigh, hIMURightKnee;
#endif
