#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include "common.h"
#include "system_periphrals.h"
#include "usb.h"

enum IMU_Operation_Mode
{
  IMU_MODE_NDOF,
  IMU_MODE_GYROONLY,
  IMU_MODE_ACCONLY
};

enum EXOSKELETON_SystemID_Tasks
{
  EXOSKELETON_SYSTEMID_TASK_FREE,
  EXOSKELETON_SYSTEMID_TASK_START,
  EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START,
  EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_POSITIONING,
  EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING,
  EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START,
  EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_POSITIONING,
  EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING,
  EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS,
  EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS,
  EXOSKELETON_SYSTEMID_TASK_END
};

typedef struct
{
  enum EXOSKELETON_SystemID_Tasks curTask;
  float kneeProfilingFreq;
  float kneeProfilingAmp;
  float hipProfilingFreq;
  float hipProfilingAmp;
  uint32_t kneeProfilingTime;
  uint32_t hipProfilingTime;
  
  uint8_t ifIdentified;
  union FloatUInt8 sysIDResults_a1;
  union FloatUInt8 sysIDResults_m1;
  union FloatUInt8 sysIDResults_a2;
  union FloatUInt8 sysIDResults_m2;
}Exoskeleton_SystemIDHandle;

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
  uint32_t              CANID_SET_MODE_ACCONLY;
  //Low pass filter
  float                 lpfCutOffFrequency;
  float                 lpfDuration;
  float                 lpfAlpha;
  float                 lpfAccXFilteredPrevious;
  float                 lpfAccYFilteredPrevious;
  float                 lpfAccZFilteredPrevious;
}BNO055Handle;

void EXOSKELETON_Init(void);
void EXOSKELETON_GetIMUFeedbackLiAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackGyro(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackQuaternion(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackStatus(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_SetIMUMode_9_DOF(BNO055Handle* himu);
void EXOSKELETON_SetIMUMode_GYRO_Only(BNO055Handle* himu);
void EXOSKELETON_SetIMUMode_ACC_Only(BNO055Handle* himu);
void EXOSKELETON_GetIMUFeedbackAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetIMUFeedbackMag(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_SystemIDManager(void);
void EXOSKELETON_SystemID_Init(void);
void EXOSKELETON_SystemID_KneeJoint_MotorProfilingSinWave(AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift);
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave(AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift);
void EXOSKELETON_SystemID_Set_Datalog_Label(void);
void EXOSKELETON_SystemID_UpdateDataSlot(void);

extern BNO055Handle hIMURightThigh, hIMURightKnee;
extern Exoskeleton_SystemIDHandle hSystemID;
#endif
