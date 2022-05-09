#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include "common.h"
#include "system_periphrals.h"

enum IMU_Operation_Mode
{
  IMU_MODE_NDOF,
  IMU_MODE_GYROONLY,
  IMU_MODE_AMG
};

typedef struct
{
	union Int16UInt8 liaccX;
	union Int16UInt8 liaccY;
	union Int16UInt8 liaccZ;
  union Int16UInt8 AccX;
	union Int16UInt8 AccY;
	union Int16UInt8 AccZ;
	union Int16UInt8 gyroX;
	union Int16UInt8 gyroY;
	union Int16UInt8 gyroZ;
  union Int16UInt8 MagX;
	union Int16UInt8 MagY;
	union Int16UInt8 MagZ;
	union Int16UInt8 Quat[4];
}BNO055_Raw_Data_Struct;

typedef struct
{
	union FloatUInt8 liaccX;
	union FloatUInt8 liaccY;
	union FloatUInt8 liaccZ;
  union FloatUInt8 AccX;
	union FloatUInt8 AccY;
	union FloatUInt8 AccZ;
	union FloatUInt8 gyroX;
	union FloatUInt8 gyroY;
	union FloatUInt8 gyroZ;
  union FloatUInt8 MagX;
	union FloatUInt8 MagY;
	union FloatUInt8 MagZ;
	union FloatUInt8 Quat[4];
}BNO055_Parsed_Data_Struct;

typedef struct
{
	uint8_t                   operationMode;
  enum IMU_Operation_Mode   operationModeENUM;
	uint8_t										calibStatus;
	uint8_t										deadCount;
	BNO055_Raw_Data_Struct	  rawData;
  BNO055_Parsed_Data_Struct parsedData;
  
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
  //CAN IDs
  uint32_t              CANID_SET_MODE_NDOF;
  uint32_t              CANID_SET_MODE_GYROONLY;
  uint32_t              CANID_SET_MODE_AMG;
}BNO055Handle;

void EXOSKELETON_Init(void);
void EXOSKELETON_GetIMUFeedbackLiAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackGyro(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackQuaternion(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackStatus(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_SetIMUMode_9_DOF(BNO055Handle* himu);
void EXOSKELETON_SetIMUMode_GYRO_Only(BNO055Handle* himu);
void EXOSKELETON_SetIMUMode_AMG(BNO055Handle* himu);
void EXOSKELETON_GetIMUFeedbackAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackMag(BNO055Handle* himu, uint8_t data[]);

extern BNO055Handle hIMURightThigh, hIMURightKnee;
#endif
