#ifndef TMOTOR_AK10_9_V2_H
#define TMOTOR_AK10_9_V2_H

#include "main.h"
#include "system_periphrals.h"
#include "can_bus.h"
#include "my_math.h"
#include <math.h>

#define AK10_9_SPECIAL_COMMAND_ENABLE_MOTOR  0U
#define AK10_9_SPECIAL_COMMAND_DISABLE_MOTOR 1U
#define AK10_9_SPECIAL_COMMAND_ZEROING_MOTOR 2U

#define SIZE_OF_MOVING_ACC_AVG_BUFFER        25U
enum AK10_9_Status
{
  AK10_9_Online,
  AK10_9_Offline
};

enum ControlModeDMFW
{
  AK10_9_DM_FW_MODE_MIT,
  AK10_9_DM_FW_MODE_POSITION,
  AK10_9_DM_FW_MODE_VELOCITY,
};

typedef struct
{
  CAN_HandleTypeDef*    hcan;
  uint32_t              canID;
  int8_t                temperature;
  uint8_t               errorCode;
  enum AK10_9_Status    status;
  
  float                 kt;
  float                 posOffset;
  float                 posDirectionCorrection;
  union FloatUInt8      setCurrent;
  union FloatUInt8      setPosition;
  union FloatUInt8      setVelocity;
  union FloatUInt8      realCurrent;
  union FloatUInt8      realPosition;
  union FloatUInt8      realPositionOffseted;
  union FloatUInt8      realPositionOffsetedRad;
  union FloatUInt8      realVelocityPresent;
  union FloatUInt8      realVelocityPresentRad;
  union FloatUInt8      realVelocityPrevious[2];
  union FloatUInt8      realTorque;
  union FloatUInt8      setAcceleration;
  union FloatUInt8      setAcceleration_ByRealPosition;
  union FloatUInt8      realAccelerationRaw;
  union FloatUInt8      realAccelerationFiltered;
  union FloatUInt8      realAccelerationFilteredRad;
  union FloatUInt8      kp, kd;
  uint8_t               ifCustomizedPositionSpeedControlFinished;
  //For acceleration estimation//
  //Moving average value method//
  float                 accAverage;
  float                 accAverageBuf[SIZE_OF_MOVING_ACC_AVG_BUFFER];
  uint8_t               accAvgPtr;
  ///////////////////////////////
  //Low pass filter method//
  float                 realAccelerationFilteredPrevious;
  float                 cutOffFrequency;
  float                 alpha;
  float                 timeDuration;
  //////////////////////////
  //Butterworth filter method//
  float                 a2Butter, a3Butter, b1Butter, b2Butter, b3Butter;
  float                 realAccelerationFilteredPreviousButter[2];
  float                 realAccelerationRawPreviousButter[2];
  /////////////////////////////
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t*             pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
  //CAN BUS receive
  uint8_t               rxBuf[8];
  uint32_t              rxFifo;
  CAN_FilterTypeDef     rxFilter;
  uint32_t              lastReceivedTime;
}AK10_9HandleCubaMarsFW;

typedef struct
{
  CAN_HandleTypeDef*    hcan;
  uint8_t               canID;
  enum AK10_9_Status    status;
  enum ControlModeDMFW  controlMode;
  
  float                 kt;
  float                 posOffset;
  float                 posDirectionCorrection;
  union FloatUInt8      setCurrent;
  union FloatUInt8      setPosition;
  union FloatUInt8      setVelocity;
  union FloatUInt8      realCurrent;
  union FloatUInt8      realPositionRad;
  union FloatUInt8      realPositionDeg;
  union FloatUInt8      realPositionOffseted;
  union FloatUInt8      realPositionOffsetedRad;
  union FloatUInt8      realVelocityPresent;
  union FloatUInt8      realVelocityPresentRad;
  union FloatUInt8      realVelocityPrevious[2];
  union FloatUInt8      realTorque;
  union FloatUInt8      setAcceleration;
  union FloatUInt8      setAcceleration_ByRealPosition;
  union FloatUInt8      realAccelerationRaw;
  union FloatUInt8      realAccelerationFiltered;
  union FloatUInt8      realAccelerationFilteredRad;
  union FloatUInt8      kp, kd;
  uint8_t               ifCustomizedPositionSpeedControlFinished;
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t*             pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
  //CAN BUS receive
  uint8_t               rxBuf[8];
  uint32_t              rxFifo;
  CAN_FilterTypeDef     rxFilter;
  uint32_t              lastReceivedTime;
  //Butterworth filter//
  float                 a2Butter, a3Butter, b1Butter, b2Butter, b3Butter;
  float                 realAccelerationFilteredPreviousButter[2];
  float                 realAccelerationRawPreviousButter[2];
  /////////////////////////////
}AK10_9HandleDMFW;

void AK10_9_V2_Init(void);

void AK10_9_ServoMode_CurrentControl(AK10_9HandleCubaMarsFW* hmotor, float current);
void AK10_9_ServoMode_VelocityControl(AK10_9HandleCubaMarsFW* hmotor, float speed);
void AK10_9_ServoMode_PositionControl(AK10_9HandleCubaMarsFW* hmotor, float position);
void AK10_9_ServoMode_PositionSpeenControlCustomized(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float loop_duration);
void AK10_9_ServoMode_PositionControlWithOffset(AK10_9HandleCubaMarsFW* hmotor, float position);
void AK10_9_ServoMode_PositionSpeenControlCustomizedWithOffset(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float loop_duration);
void AK10_9_ServoMode_PositionSpeedControl(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, int16_t acceleration);
void AK10_9_ServoMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubaMarsFW* hmotor, uint8_t rxbuf[]);
void AK10_9_ServoMode_Zeroing(AK10_9HandleCubaMarsFW* hmotor);
void AK10_9_MITMode_EnableMotor(AK10_9HandleCubaMarsFW* hmotor);
void AK10_9_MITMode_DisableMotor(AK10_9HandleCubaMarsFW* hmotor);
void AK10_9_MITMode_Zeroing(AK10_9HandleCubaMarsFW* hmotor);
void AK10_9_MITModeControl_Deg(AK10_9HandleCubaMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_MITModeControl_Rad(AK10_9HandleCubaMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_MITMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubaMarsFW* hmotor, uint8_t rxbuf[]);
void AK10_9_MotorStatusMonitor(AK10_9HandleCubaMarsFW* hmotor);

void AK10_9_DMFW_EnableMotor(AK10_9HandleDMFW* hmotor);
void AK10_9_DMFW_DisableMotor(AK10_9HandleDMFW* hmotor);
void AK10_9_DMFW_Zeroing(AK10_9HandleDMFW* hmotor);
void AK10_9_DMFW_MITModeControl_Rad(AK10_9HandleDMFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_DMFW_MITModeControl_Deg(AK10_9HandleDMFW* hmotor, float pos, float vel, float kp, float kd, float iq);
void AK10_9_DMFW_PositionVelocityControl(AK10_9HandleDMFW* hmotor, float pos, float vel);
void AK10_9_DMFW_VelocityControl(AK10_9HandleDMFW* hmotor, float vel);
void AK10_9_DMFW_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleDMFW* hmotor, uint8_t rxbuf[]);
uint16_t FloatToUint(float x, float x_min, float x_max, uint16_t bits);
float    UintToFloat(uint16_t x_int, float x_min, float x_max, uint16_t bits);





#endif
