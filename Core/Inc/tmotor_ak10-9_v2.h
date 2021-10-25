#ifndef TMOTOR_AK10_9_V2_H
#define TMOTOR_AK10_9_V2_H

#include "main.h"
#include "system_periphrals.h"

typedef struct
{
  CAN_HandleTypeDef*  hcan;
  uint8_t             canID;
}AK10_9Handle;

typedef enum
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
}ServoMotorMode_CAN_PACKET_ID;

void AK10_9_V2_Init(void);











#endif
