#ifndef TMOTOR_AK10_9_V2_H
#define TMOTOR_AK10_9_V2_H

#include "main.h"
#include "system_periphrals.h"
#include "can_bus.h"
#include "my_math.h"

#define AK10_9_SPECIAL_COMMAND_ENABLE_MOTOR 0U
#define AK10_9_SPECIAL_COMMAND_DISABLE_MOTOR 1U
#define AK10_9_SPECIAL_COMMAND_ZEROING_MOTOR 2U

typedef struct
{
  CAN_HandleTypeDef*    hcan;
  uint32_t              canID;
  int8_t                temperature;
  uint8_t               errorCode;
  
  union FloatUInt8      setCurrent;
  union FloatUInt8      setPosition;
  union FloatUInt8      setVelocity;
  union FloatUInt8      setAcceleration;
  union FloatUInt8      realCurrent;
  union FloatUInt8      realPosition;
  union FloatUInt8      realVelocity;
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t*             pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
  //CAN BUS receive
  uint8_t               rxBuf[8];
  uint32_t              rxFifo;
  CAN_FilterTypeDef     rxFilter;
}AK10_9Handle;

void AK10_9_V2_Init(void);

void AK10_9_ServoMode_CurrentControl(AK10_9Handle* hmotor, float current);
void AK10_9_ServoMode_VelocityControl(AK10_9Handle* hmotor, float speed);
void AK10_9_ServoMode_PositionControl(AK10_9Handle* hmotor, float position);
void AK10_9_ServoMode_PositionSpeedControl(AK10_9Handle* hmotor, float position, float speed, int16_t acceleration);
void AK10_9_ServoMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9Handle* hmotor, uint8_t rxbuf[]);
void AK10_9_ServoMode_Zeroing(AK10_9Handle* hmotor);
void AK10_9_SpecialCommand(AK10_9Handle* hmotor, uint8_t specialCmd);






#endif
