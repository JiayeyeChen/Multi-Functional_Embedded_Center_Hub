#include "tmotor_ak10-9_v2.h"

const uint8_t can_special_msg_enable_motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
const uint8_t can_special_msg_disable_motor[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
const uint8_t can_special_msg_seroing[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};

enum ServoMotorMode_CAN_PACKET_ID
{
  SERVO_CAN_PACKET_SET_DUTY = 0,
  SERVO_CAN_PACKET_SET_CURRENT,
  SERVO_CAN_PACKET_SET_CURRENT_BRAKE,
  SERVO_CAN_PACKET_SET_RPM,
  SERVO_CAN_PACKET_SET_POS,
  SERVO_CAN_PACKET_SET_ORIGIN_HERE,
  SERVO_CAN_PACKET_SET_ADRC_GAIN,
  SERVO_CAN_PACKET_SET_ENC_OFFSET,
  SERVO_CAN_PACKET_SET_ELEC_ZERO,
  SERVO_CAN_PACKET_SET_POS_SPD = 39,
  SERVO_CAN_PACKET_SET_ZESC = 40,
};

AK10_9Handle hAK10_1;

void AK10_9_V2_Init(void)
{
  hAK10_1.hcan = &hcan1;
  hAK10_1.canID = MY_CAN_ID_TMotor_Testing;
  hAK10_1.hTxHeader.IDE = CAN_ID_STD;
  hAK10_1.hTxHeader.RTR = CAN_RTR_DATA;
  hAK10_1.hTxHeader.StdId = (uint32_t)hAK10_1.canID;
  hAK10_1.hTxHeader.ExtId = (uint32_t)hAK10_1.canID;
}

void AK10_9_ServoMode_CurrentControl(AK10_9Handle* hmotor, float current)
{
  hmotor->setCurrent.f = current;
  hmotor->hTxHeader.IDE = CAN_ID_EXT;
  hmotor->hTxHeader.DLC = 4;
  hmotor->hTxHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_CURRENT << 8) | (uint32_t)hAK10_1.canID;
  union Int32UInt8 temCurrent;
  temCurrent.b32 = (int32_t)(hmotor->setCurrent.f * 1000.0f);
  hmotor->txBuf[0] = temCurrent.b8[0];
  hmotor->txBuf[1] = temCurrent.b8[1];
  hmotor->txBuf[2] = temCurrent.b8[2];
  hmotor->txBuf[3] = temCurrent.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_VelocityControl(AK10_9Handle* hmotor, float speed)
{
  hmotor->setVelocity.f = speed;
  hmotor->hTxHeader.IDE = CAN_ID_EXT;
  hmotor->hTxHeader.DLC = 4;
  hmotor->hTxHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_RPM << 8) | (uint32_t)hAK10_1.canID;
  
  union Int32UInt8 temRPM;
  temRPM.b32 = (int32_t)hmotor->setVelocity.f * 120.0f * pi;
  hmotor->txBuf[0] = temRPM.b8[0];
  hmotor->txBuf[1] = temRPM.b8[1];
  hmotor->txBuf[2] = temRPM.b8[2];
  hmotor->txBuf[3] = temRPM.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_PositionControl(AK10_9Handle* hmotor, float position)
{
  hmotor->setPosition.f = position;
  hmotor->hTxHeader.IDE = CAN_ID_EXT;
  hmotor->hTxHeader.DLC = 4;
  hmotor->hTxHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_POS << 8) | (uint32_t)hAK10_1.canID;
  
  union Int32UInt8 temPOS;
  temPOS.b32 = (int32_t)(hmotor->setPosition.f * 1000000.0f);
  hmotor->txBuf[0] = temPOS.b8[0];
  hmotor->txBuf[1] = temPOS.b8[1];
  hmotor->txBuf[2] = temPOS.b8[2];
  hmotor->txBuf[3] = temPOS.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_SpecialCommand(AK10_9Handle* hmotor, uint8_t specialCmd)
{
  hmotor->hTxHeader.DLC = 8;
  hmotor->hTxHeader.IDE = CAN_ID_STD;
  if (specialCmd == AK10_9_SPECIAL_COMMAND_ENABLE_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, (uint8_t*)can_special_msg_enable_motor, hmotor->pTxMailbox);
  else if (specialCmd == AK10_9_SPECIAL_COMMAND_DISABLE_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, (uint8_t*)can_special_msg_disable_motor, hmotor->pTxMailbox);
  else if (specialCmd == AK10_9_SPECIAL_COMMAND_ZEROING_MOTOR)
    HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->hTxHeader, (uint8_t*)can_special_msg_seroing, hmotor->pTxMailbox);
}
