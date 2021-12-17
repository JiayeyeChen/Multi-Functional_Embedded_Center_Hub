#include "tmotor_ak10-9_v2.h"

const uint8_t can_special_msg_enable_motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t can_special_msg_disable_motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t can_special_msg_seroing[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

//__fp16 halfFloatTest1;

enum ServoMotorMode_CAN_PACKET_ID
{
  SERVO_CAN_PACKET_SET_DUTY = 0,
  SERVO_CAN_PACKET_SET_CURRENT,
  SERVO_CAN_PACKET_SET_CURRENT_BRAKE,
  SERVO_CAN_PACKET_SET_RPM,
  SERVO_CAN_PACKET_SET_POS,
  SERVO_CAN_PACKET_SET_ORIGIN_HERE,
  SERVO_CAN_PACKET_SET_POS_SPD
};

//float current: -60A~60A
void AK10_9_ServoMode_CurrentControl(AK10_9Handle* hmotor, float current)
{
  hmotor->setCurrent.f = current;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_CURRENT << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  union Int32UInt8 temCurrent;
  temCurrent.b32 = (int32_t)(hmotor->setCurrent.f * 1000.0f);
  hmotor->txBuf[0] = temCurrent.b8[3];
  hmotor->txBuf[1] = temCurrent.b8[2];
  hmotor->txBuf[2] = temCurrent.b8[1];
  hmotor->txBuf[3] = temCurrent.b8[0];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

//float speed: -10000epm~10000epm
void AK10_9_ServoMode_VelocityControl(AK10_9Handle* hmotor, float speed)
{
  hmotor->setVelocity.f = speed;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_RPM << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temSPD;
  temSPD.b32 = (int32_t)hmotor->setVelocity.f;
  hmotor->txBuf[0] = temSPD.b8[0];
  hmotor->txBuf[1] = temSPD.b8[1];
  hmotor->txBuf[2] = temSPD.b8[2];
  hmotor->txBuf[3] = temSPD.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

//float position: -3600deg~3600deg
void AK10_9_ServoMode_PositionControl(AK10_9Handle* hmotor, float position)
{
  hmotor->setPosition.f = position;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_POS << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temPOS;
  temPOS.b32 = (int32_t)(hmotor->setPosition.f * 10000.0f);
  hmotor->txBuf[0] = temPOS.b8[3];
  hmotor->txBuf[1] = temPOS.b8[2];
  hmotor->txBuf[2] = temPOS.b8[1];
  hmotor->txBuf[3] = temPOS.b8[0];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_PositionSpeedControl(AK10_9Handle* hmotor, float position, float speed, int16_t acceleration)
{
  hmotor->setPosition.f = position;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_POS_SPD << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temPOS;
  temPOS.b32 = (int32_t)hmotor->setPosition.f * 10000.0f;
  hmotor->txBuf[0] = temPOS.b8[3];
  hmotor->txBuf[1] = temPOS.b8[2];
  hmotor->txBuf[2] = temPOS.b8[1];
  hmotor->txBuf[3] = temPOS.b8[0];
  
  union Int16UInt8 temSPD;
  temSPD.b16 = (int16_t)(speed * 60.0f * 21.0f * 9.0f / 3600.0f);
  hmotor->txBuf[4] = temSPD.b8[1];
  hmotor->txBuf[5] = temSPD.b8[0];
  hmotor->txBuf[6] = (uint8_t)((acceleration >> 8) & 0xFF);
  hmotor->txBuf[7] = (uint8_t)(acceleration * 0xFF);
  
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9Handle* hmotor, uint8_t rxbuf[])
{
  hmotor->lastReceivedTime = HAL_GetTick();
  if (rxheader->ExtId == hmotor->canID)
    memcpy(hmotor->rxBuf, rxbuf, 8);
  hmotor->realPosition.f = (float)((int16_t)(hmotor->rxBuf[0] << 8 | hmotor->rxBuf[1]));
  hmotor->realPosition.f /= 10.0f;
  hmotor->realVelocity.f = (float)((int16_t)(hmotor->rxBuf[2] << 8 | hmotor->rxBuf[3]));
  hmotor->realVelocity.f = (hmotor->realVelocity.f * 3600.0f / 60.0f) / (21.0f * 9.0f);
  hmotor->realCurrent.f = (float)((int16_t)(hmotor->rxBuf[4] << 8 | hmotor->rxBuf[5]));
  hmotor->realCurrent.f /= 100.0f;
  hmotor->temperature = (int8_t)hmotor->rxBuf[6];
  hmotor->errorCode = hmotor->rxBuf[7];
}

void AK10_9_ServoMode_Zeroing(AK10_9Handle* hmotor)
{
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 1;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_ORIGIN_HERE << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  hmotor->txBuf[0] = 1;
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_SpecialCommand(AK10_9Handle* hmotor, uint8_t specialCmd)
{
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = CAN_ID_STD;
  if (specialCmd == AK10_9_SPECIAL_COMMAND_ENABLE_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, (uint8_t*)can_special_msg_enable_motor, hmotor->pTxMailbox);
  else if (specialCmd == AK10_9_SPECIAL_COMMAND_DISABLE_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, (uint8_t*)can_special_msg_disable_motor, hmotor->pTxMailbox);
  else if (specialCmd == AK10_9_SPECIAL_COMMAND_ZEROING_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, (uint8_t*)can_special_msg_seroing, hmotor->pTxMailbox);
}

void AK10_9_MotorStatusMonitor(AK10_9Handle* hmotor)
{
  if ((HAL_GetTick() - hmotor->lastReceivedTime) > 3)
    hmotor->status = AK10_9_Offline;
  else
    hmotor->status = AK10_9_Online;
}
