/** @file   encoder.h
 *  @brief  Header file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#ifndef MRBA3_ENCODER_H
#define MRBA3_ENCODER_H

#include "common.h"
#include "can_bus.h"

#define BRITER_ENCODER_CAN_COMMAND_READ_VALUE               0x01
#define BRITER_ENCODER_CAN_COMMAND_SET_ID                   0x02
#define BRITER_ENCODER_CAN_COMMAND_SET_CAN_BAUDRATE         0x03
#define BRITER_ENCODER_CAN_COMMAND_SET_MODE                 0x04
#define BRITER_ENCODER_CAN_COMMAND_SET_AUTO_FEEDBACK_TIME   0x05
#define BRITER_ENCODER_CAN_COMMAND_ZEROING                  0x06
#define BRITER_ENCODER_CAN_COMMAND_SET_DIRECTION            0x07

#define BRITER_ENCODER_DIRECTION_CLOCKWISE                  0x00
#define BRITER_ENCODER_DIRECTION_COUNTERCLOCKWISE           0x01

#define SIZE_OF_ANGLE_AVG_BUF                               10

typedef struct
{
  uint8_t               canAddress;
  CAN_HandleTypeDef*    hcan;
  CAN_RxHeaderTypeDef   canRxHeader;
  CAN_FilterTypeDef     canRxFilter;
  CAN_TxHeaderTypeDef   canTxHeader;
  uint32_t              canMailbox;
	union UInt32UInt8     angleRaw;
  union FloatUInt8      angleDeg;
  union FloatUInt8      angleRad;
  union FloatUInt8      speedDeg;
  union FloatUInt8      speedRad;
  float                 speedRatio;
  float                 speedCalAngleBuf[SIZE_OF_ANGLE_AVG_BUF];
  uint8_t               speedCalAngleBufPtr;
  float                 speedCalAngleAvgPresent;
  float                 speedCalAngleAvgPrevious;
  uint8_t               txBuf[8];
  uint32_t              lastLegitRxTimestamp;
}EncoderHandle;

void ENCODER_Init(void);

void ENCODER_ReadAngleRequest(EncoderHandle* hencoder);

void ENCODER_GetAngle(EncoderHandle* hencoder, uint8_t* rx_buf);

void ENCODER_CalculateSpeed(EncoderHandle* hencoder, float time_interval);

void ENCODER_SetZeroPosition(EncoderHandle* hencoder);

void ENCODER_Set1MHzCanBaudrate(EncoderHandle* hencoder);

void ENCODER_SetCanID(EncoderHandle* hencoder, uint8_t id);

void ENCODER_TimeoutDetect(EncoderHandle* hencoder, uint32_t ms);

void ENCODER_SetDirection(EncoderHandle* hencoder, uint8_t briter_encoder_direction);

void ENCODER_SetAutoFeedbackMode(EncoderHandle* hencoder);

void ENCODER_SetManualFeedbackMode(EncoderHandle* hencoder);

void ENCODER_SetAutoFeedbackRate(EncoderHandle* hencoder, uint16_t us);

extern EncoderHandle hEncoderRightWheel, hEncoderLeftWheel, hEncoderLeftPull;

#endif
