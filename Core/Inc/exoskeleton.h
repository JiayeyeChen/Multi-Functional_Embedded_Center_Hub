#ifndef EXOSKELETON_H
#define EXOSKELETON_H

#include "common.h"
#include "system_periphrals.h"
#include "usb.h"

//#define HIP_JOINT_LEARNING_USE_CURRENT_CONTROL

#define EXOSKELETON_GRAVITATIONAL_ACCELERATION     9.781f
#define EXOSKELETON_SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME 1000
#define EXOSKELETON_SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT 180.0f
#define EXOSKELETON_SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT 90.0f
#define EXOSKELETON_SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT 180.0f
#define EXOSKELETON_SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT 90.0f

#define EXOSKELETON_HIP_IMU_ACC_X_OFFSET    -0.165469378f;
#define EXOSKELETON_HIP_IMU_ACC_Y_OFFSET    -0.0572346263f;
#define EXOSKELETON_HIP_IMU_ACC_Z_OFFSET    9.77716637f - EXOSKELETON_GRAVITATIONAL_ACCELERATION;

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

enum EXOSKELETON_Main_Tasks
{
  EXOSKELETON_MAIN_TASK_FREE,
  EXOSKELETON_MAIN_TASK_SYSTEM_ID,
  EXOSKELETON_MAIN_TASK_GRAVITY_COMPENSATION,
  EXOSKELETON_MAIN_TASK_AUGMENTATION_CONTROL,
  EXOSKELETON_MAIN_TASK_MOTOR_PROFILING,
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
  union FloatUInt8 sysIDResults_J2;
  union FloatUInt8 sysIDResults_X2;
  union FloatUInt8 sysIDResults_J1;
  union FloatUInt8 sysIDResults_X1;
}Exoskeleton_SystemIDHandle;

typedef struct
{
  float throttleHip, throttleKnee;
  uint8_t ifGravityCompensationStarted;
  union FloatUInt8 torqueDesiredHip;
  union FloatUInt8 torqueDesiredKnee;
}Exoskeleton_GravityCompensation;

typedef struct
{
  uint8_t          ifEstimating;
  union FloatUInt8 muscularTorqueHip;
  union FloatUInt8 muscularTorqueKnee;
}Exoskeleton_MuscularTorqueEstimationHandle;

typedef struct
{
  uint8_t ifAugmentedControl;
  float hipJointAugmentedControlThrottle;
  float kneeJointAugmentedControlThrottle;
}Exoskeleton_AugmentedControlHandle;

typedef struct
{
  uint8_t ifMotorProfiling;
  uint32_t motorProfilingStartingTimestamp;
  float (*HipGaitFunc) (float);
  float (*KneeGaitFunc) (float);
}Exoskeleton_MotorProfilingHandle;

typedef struct
{
  union FloatUInt8 L1;
  enum EXOSKELETON_Main_Tasks mainTask;
  Exoskeleton_SystemIDHandle* hsysid;
  Exoskeleton_GravityCompensation* hgravitycompensation;
  Exoskeleton_MuscularTorqueEstimationHandle* hmusculartorque;
  Exoskeleton_AugmentedControlHandle* haugmentedcontrol;
  Exoskeleton_MotorProfilingHandle* hmotorprofiling;
}ExoskeletonHandle;

typedef struct
{
	union Int16UInt8 liaccX;
	union Int16UInt8 liaccY;
	union Int16UInt8 liaccZ;
  union Int16UInt8 AccX;
	union Int16UInt8 AccY;
	union Int16UInt8 AccZ;
  union Int16UInt8 grvX;
  union Int16UInt8 grvY;
  union Int16UInt8 grvZ;
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
  union FloatUInt8 grvX;
  union FloatUInt8 grvY;
  union FloatUInt8 grvZ;
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
}BNO055Handle;

typedef struct
{
  CAN_HandleTypeDef*    hcan;
  uint8_t							  bno055CalibStatus;
  union FloatUInt8      xAcc, yAcc, zAcc, accNorm;
  AveragerHandle        xAccAvg, yAccAvg, zAccAvg;
  union FloatUInt8      xGrv, yGrv, zGrv, grvNorm;
  union FloatUInt8      xGrvNormalized, yGrvNormalized, zGrvNormalized;
  union FloatUInt8      xLiAcc, yLiAcc, zLiAcc;
}JointAccelerationIMUHandle;

void EXOSKELETON_Init(void);
void EXOSKELETON_CommonDatalogManager(void);
void EXOSKELETON_UpdateCommonDataSlot(void);
void EXOSKELETON_Set_Common_Datalog_Label(void);
void EXOSKELETON_GetBNO055FeedbackLiAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetBNO055FeedbackGyro(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetBNO055FeedbackQuaternion(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetBNO055FeedbackStatus(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_SetBNO055Mode_9_DOF(BNO055Handle* himu);
void EXOSKELETON_SetBNO055Mode_GYRO_Only(BNO055Handle* himu);
void EXOSKELETON_SetBNO055Mode_ACC_Only(BNO055Handle* himu);
void EXOSKELETON_GetBNO055FeedbackAcc(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetBNO055FeedbackMag(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetBNO055FeedbackGrv(BNO055Handle* himu, uint8_t data[]);
void EXOSKELETON_GetJointAccelerationIMUFeedback_X_Data(JointAccelerationIMUHandle* himu, uint8_t data[]);
void EXOSKELETON_GetJointAccelerationIMUFeedback_Y_Data(JointAccelerationIMUHandle* himu, uint8_t data[]);
void EXOSKELETON_GetJointAccelerationIMUFeedback_Z_Data(JointAccelerationIMUHandle* himu, uint8_t data[]);
void EXOSKELETON_GetJointAccelerationIMUFeedback_BNO055Status(JointAccelerationIMUHandle* himu, uint8_t data[]);
void EXOSKELETON_SystemIDManager(void);
void EXOSKELETON_SystemID_Init(void);
void EXOSKELETON_SystemID_KneeJoint_MotorProfilingSinWave(AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift);
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_PositionControl(AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift);
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_CurrentControl(AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift);
void EXOSKELETON_SystemID_Set_Datalog_Label(void);
void EXOSKELETON_SystemID_UpdateDataSlot(void);
void EXOSKELETON_GravityCompensation_Init(Exoskeleton_GravityCompensation* hgravitycompensation);
void EXOSKELETON_CentreControl(void);
void EXOSKELETON_GravityCompemsationManager(void);
void EXOSKELETON_MuscularTorqueCalculation(ExoskeletonHandle* hexoskeleton);
void EXOSKELETON_AugmentedControlManager(void);
float EXOSKELETON_HipGait1(float sec);
float EXOSKELETON_HipGait2(float sec);
void EXOSKELETON_Motor_Durability_Test_Set_Datalog_Label(void);

extern BNO055Handle hIMUTorso;
extern JointAccelerationIMUHandle hIMUHip, hIMUKnee;
extern JointAccelerationIMUHandle* hCustomizedIMUPtr;
extern Exoskeleton_SystemIDHandle hSystemID;
extern Exoskeleton_GravityCompensation hGravityCompensation;
extern ExoskeletonHandle hExoskeleton;
extern Exoskeleton_AugmentedControlHandle hAugmentedControl;
#endif
