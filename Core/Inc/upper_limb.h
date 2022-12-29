#ifndef UPPER_LIMB_H
#define UPPER_LIMB_H

#include "common.h"
#include "can_bus.h"
#include "ak10-9_v2_testing.h"
#include "encoder.h"

#define UPPERLIMB_L1  0.1185f
#define UPPERLIMB_L31 0.1f
#define UPPERLIMB_L21 0.43f
#define UPPERLIMB_L4  0.26155f
#define UPPERLIMB_Lf  0.1345f

enum UpperLimbMainTask
{
  UPPERLIMB_MAIN_TASK_NONE,
  UPPERLIMB_MAIN_TASL_ADMITTANCE_CONTROL,
  UPPERLIMB_MAIN_TASK_MANUAL_CONTROL,
  UPPERLIMB_MAIN_TASK_PID_VELOCITY_CONTROL
};

typedef struct
{
  union Int16UInt8 forceXRaw, forceYRaw, forceZRaw;
  union FloatUInt8 forceX, forceY, forceZ;
  union FloatUInt8 encoderAngleDeg, encoderAngleRad;
  enum UpperLimbMainTask mainTask;
  //CAN Bus
  CAN_TxHeaderTypeDef txHeader;
  CAN_HandleTypeDef* hcan;
  uint32_t txMailBox;
  uint8_t canTxBuf[8];
  //Forward Kinematics
  union FloatUInt8 endPosX, endPosY, endPosZ;
  union FloatUInt8 q21Rad, q31Rad, q1Rad;
  union FloatUInt8 q21Deg, q31Deg, q1Deg;
  union FloatUInt8 q21DotRad, q31DotRad, q1DotRad;
  union FloatUInt8 q21DotDeg, q31DotDeg, q1DotDeg;
  union FloatUInt8 q4Deg, q4Rad;
  //Desired Control
  union FloatUInt8 endPosDesX, endPosDesY, endPosDesZ;
  union FloatUInt8 q21RadDes, q31RadDes, q1RadDes;
  union FloatUInt8 q21DegDes, q31DegDes, q1DegDes;
  union FloatUInt8 q21DotRadDes, q31DotRadDes, q1DotRadDes;
  union FloatUInt8 q21DotDegDes, q31DotDegDes, q1DotDegDes;
  //Jacobian
  float detJ;
  float delta1;
  float J[3][3];
  float invJ[3][3];
}UpperLimbHandle;

void UPPERLIMB_Init(void);
void UPPERLIMB_CANGetForceFeedback(UpperLimbHandle* hupperlimb, uint8_t data[]);
void UPPERLIMB_CANRequestForceData(UpperLimbHandle* hupperlimb);
void UPPERLIMB_ControlCenter(void);
void UPPERLIMB_ForwardKinematics(UpperLimbHandle* hupperlimb);
void UPPERLIMB_InverseKinematics(UpperLimbHandle* hupperlimb);
void UPPERLIMB_Calculate_Jacobian(UpperLimbHandle* hupperlimb);
void UPPERLIMB_EndEffectorSpeedControl(UpperLimbHandle* hupperlimb);

extern UpperLimbHandle hUpperLimb;
extern float pid_speed_control_set_val;
#endif


