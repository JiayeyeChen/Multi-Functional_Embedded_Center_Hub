/** @file   encoder.c
 *  @brief  Source file of Briter multi-turn CAN bus encoder.
 *  @author Rehabilitation Research Institute of Singapore / MRBA Team
 */

#include "encoder.h"
#include "system_periphrals.h"

EncoderHandle hEncoderRightWheel, hEncoderLeftWheel, hEncoderLeftPull, \
              hEncoderUpperLimb, hEncoderLeftTurn, hEncoderRightPull, hEncoderRightTurn;
EncoderHandle* hEncoderPtr;
uint8_t encoderSelectPtr = 0;
uint8_t ifRequestRead = 0;

void ENCODER_Init(void)
{
  hEncoderLeftPull.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderLeftPull.hcan = &hcan2;
  hEncoderLeftPull.canAddress = CAN_ID_ENCODER_LEFT_PULL;
  hEncoderLeftPull.speedCalAngleBufPtr = 0;
  memset(hEncoderLeftPull.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderLeftPull.speedRatio = 1.0f;
  
  hEncoderLeftTurn.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderLeftTurn.hcan = &hcan2;
  hEncoderLeftTurn.canAddress = CAN_ID_ENCODER_LEFT_TURN;
  hEncoderLeftTurn.speedCalAngleBufPtr = 0;
  memset(hEncoderLeftTurn.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderLeftTurn.speedRatio = 1.0f;
  /////////////////////////////////////////////////////////
  hEncoderRightPull.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderRightPull.hcan = &hcan2;
  hEncoderRightPull.canAddress = CAN_ID_ENCODER_RIGHT_PULL;
  hEncoderRightPull.speedCalAngleBufPtr = 0;
  memset(hEncoderRightPull.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderRightPull.speedRatio = 1.0f;
  
  hEncoderRightTurn.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderRightTurn.hcan = &hcan2;
  hEncoderRightTurn.canAddress = CAN_ID_ENCODER_RIGHT_TURN;
  hEncoderRightTurn.speedCalAngleBufPtr = 0;
  memset(hEncoderRightTurn.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderRightTurn.speedRatio = 1.0f;
  
  
  hEncoderLeftWheel.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderLeftWheel.hcan = &hcan2;
  hEncoderLeftWheel.canAddress = CAN_ID_ENCODER_LEFT_WHEEL;
  hEncoderLeftWheel.speedCalAngleBufPtr = 0;
  memset(hEncoderLeftWheel.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderLeftWheel.speedRatio = 1.0f;
  
  hEncoderRightWheel.lastLegitRxTimestamp = HAL_GetTick();
  hEncoderRightWheel.hcan = &hcan2;
  hEncoderRightWheel.canAddress = CAN_ID_ENCODER_RIGHT_WHEEL;
  hEncoderRightWheel.speedCalAngleBufPtr = 0;
  memset(hEncoderRightWheel.speedCalAngleBuf, 0, SIZE_OF_ANGLE_AVG_BUF);
  hEncoderRightWheel.speedRatio = 1.0f;
  
  encoderSelectPtr = 0;
  hEncoderPtr = &hEncoderLeftPull;
}


void ENCODER_ReadAngleRequest(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canAddress;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_READ_VALUE;
	hencoder->txBuf[3] = 0x00;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_GetAngle(EncoderHandle* hencoder, uint8_t* rx_buf)
{
	hencoder->angleRaw.b8[0] = *(rx_buf + 3);
	hencoder->angleRaw.b8[1] = *(rx_buf + 4);
	hencoder->angleRaw.b8[2] = *(rx_buf + 5);
	hencoder->angleRaw.b8[3] = *(rx_buf + 6);
  hencoder->angleDeg.f = ((float)hencoder->angleRaw.b32) * 8640.0f / 98304.0f;
  hencoder->angleRad.f = hencoder->angleDeg.f * pi / 180.0f;
}

void ENCODER_CalculateSpeed(EncoderHandle* hencoder, float time_interval)
{
  hencoder->speedCalAngleBuf[hencoder->speedCalAngleBufPtr++] = hencoder->angleDeg.f;
  if (hencoder->speedCalAngleBufPtr > sizeof(hencoder->speedCalAngleBuf) - 1)
    hencoder->speedCalAngleBufPtr = 0;
  
  hencoder->speedCalAngleAvgPresent = 0.0f;
  for (uint8_t i = 0; i <= SIZE_OF_ANGLE_AVG_BUF - 1; i++) //
  {
    hencoder->speedCalAngleAvgPresent += hencoder->speedCalAngleBuf[i];
  }
  hencoder->speedCalAngleAvgPresent /= SIZE_OF_ANGLE_AVG_BUF;
  hencoder->speedDeg.f = hencoder->speedRatio * (hencoder->speedCalAngleAvgPresent - hencoder->speedCalAngleAvgPrevious) / time_interval;
  hencoder->speedRad.f = hencoder->speedDeg.f * pi / 180.0f;
  hencoder->speedCalAngleAvgPrevious = hencoder->speedCalAngleAvgPresent;
}

void ENCODER_SetZeroPosition(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_ZEROING;
	hencoder->txBuf[3] = 0x00;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_Set1MHzCanBaudrate(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_CAN_BAUDRATE;
	hencoder->txBuf[3] = 0x01;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_Set500kHzCanBaudrate(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_CAN_BAUDRATE;
	hencoder->txBuf[3] = 0x00;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_SetCanID(EncoderHandle* hencoder, uint8_t id)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_ID;
	hencoder->txBuf[3] = id;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_TimeoutDetect(EncoderHandle* hencoder, uint32_t ms)
{
}

void ENCODER_SetDirection(EncoderHandle* hencoder, uint8_t briter_encoder_direction)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_DIRECTION;
	hencoder->txBuf[3] = briter_encoder_direction;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_SetAutoFeedbackMode(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_MODE;
	hencoder->txBuf[3] = 0xAA;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}
void ENCODER_SetAutoFeedbackRate(EncoderHandle* hencoder, uint16_t us)
{
  hencoder->canTxHeader.DLC = 5;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_AUTO_FEEDBACK_TIME;
  //for wheels//
//	hencoder->txBuf[4] = us & 0x00FF;
//  hencoder->txBuf[3] = (us >> 8) & 0x00FF;
  //for arms//
  hencoder->txBuf[3] = us & 0x00FF;
  hencoder->txBuf[4] = (us >> 8) & 0x00FF;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}

void ENCODER_SetManualFeedbackMode(EncoderHandle* hencoder)
{
  hencoder->canTxHeader.DLC = 4;
  hencoder->canTxHeader.IDE = 0;
  hencoder->canTxHeader.RTR = 0;
  hencoder->canTxHeader.StdId = hencoder->canAddress;
  
	hencoder->txBuf[0] = hencoder->canTxHeader.DLC;
	hencoder->txBuf[1] = hencoder->canTxHeader.StdId;
	hencoder->txBuf[2] = BRITER_ENCODER_CAN_COMMAND_SET_MODE;
	hencoder->txBuf[3] = 0x00;
	
	HAL_CAN_AddTxMessage(hencoder->hcan, &(hencoder->canTxHeader), hencoder->txBuf, &(hencoder->canMailbox));
}
