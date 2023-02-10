#include "lktech_mg_motor.h"

void LKTECH_MG_Init(LKTECH_MG_Handle* hmotor, CAN_HandleTypeDef* hcan, uint32_t motor_id, float gear_ratio)
{
  hmotor->motorID = motor_id;
  hmotor->hcan = hcan;
  hmotor->canID = 0x140 + hmotor->motorID;
	hmotor->gearRatio = gear_ratio;
  
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
	hmotor->txHeader.StdId = hmotor->canID;
}

void LKTECH_MG_SendSingleCommand(LKTECH_MG_Handle* hmotor, uint8_t command, uint8_t task_name)
{
  hmotor->task = task_name;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = command;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void LKTECH_MG_GetFeedback(LKTECH_MG_Handle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[])
{
  switch (hmotor->task)
  {
  	case LKTECH_MG_CAN_BUS_TASK_READ_PID:
      if(rxbuf[0] == LKTECH_MG_COMMAND_READ_PID)
      {
        hmotor->positionLoopKp = (float)rxbuf[2];
        hmotor->positionLoopKi = (float)rxbuf[3];
        hmotor->speedLoopKp = (float)rxbuf[4];
        hmotor->speedLoopKi = (float)rxbuf[5];
        hmotor->currentLoopKp = (float)rxbuf[6];
        hmotor->currentLoopKi = (float)rxbuf[7];
      }
  		break;
    case LKTECH_MG_CAN_BUS_TASK_READ_ACCELERATION:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_ACCELERATION)
      {
        hmotor->accelerationDeg.b8[0] = rxbuf[4];
        hmotor->accelerationDeg.b8[1] = rxbuf[5];
        hmotor->accelerationDeg.b8[2] = rxbuf[6];
        hmotor->accelerationDeg.b8[3] = rxbuf[7];
      }
      break;
      case LETECH_MG_CAN_BUS_TASK_READ_ANGLE_SINGLE_TURN:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_ANGLE_SINGLE_TURN)
      {
        hmotor->angleRaw.b8[0] = rxbuf[4];
        hmotor->angleRaw.b8[1] = rxbuf[5];
        hmotor->angleRaw.b8[2] = rxbuf[6];
        hmotor->angleRaw.b8[3] = rxbuf[7];
        hmotor->angle.f = (((float)hmotor->angleRaw.b32) * 0.01f) / hmotor->gearRatio;
      }
      break;
      case LETECH_MG_CAN_BUS_TASK_READ_ANGLE_MULTI_TURN:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_ANGLE_MULTI_TURN)
      {
        hmotor->angleMultiTurnRaw.b8[0] = rxbuf[1];
        hmotor->angleMultiTurnRaw.b8[1] = rxbuf[2];
        hmotor->angleMultiTurnRaw.b8[2] = rxbuf[3];
        hmotor->angleMultiTurnRaw.b8[3] = rxbuf[4];
        hmotor->angleMultiTurnRaw.b8[4] = rxbuf[5];
        hmotor->angleMultiTurnRaw.b8[5] = rxbuf[6];
        hmotor->angleMultiTurnRaw.b8[6] = rxbuf[7];
        hmotor->angleMultiTurn.f = ((float)hmotor->angleMultiTurnRaw.b64) * 0.01f;
      }
      break;
      case LETECH_MG_CAN_BUS_TASK_READ_CONDITION1_AND_ERROR:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_CONDITION1_N_ERROR)
      {
        hmotor->temperature.f = (float)((int8_t)rxbuf[1]);
        hmotor->voltage.f = ((float)(((uint16_t)rxbuf[3]) | (((uint16_t)rxbuf[4])<<8))) * 0.1f;
        hmotor->errorCode = rxbuf[7];
      }
      break;
      case LETECH_MG_CAN_BUS_TASK_READ_CONDITION2:
      case LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL:
      case LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL:
      case LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_1_MULTI_TURN:
      case LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_2_MULTI_TURN:
      case LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_5_INCREMENT:
      case LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_CONDITION2   | rxbuf[0] == LKTECH_MG_COMMAND_CURRENT_CONTROL   | \
					rxbuf[0] == LKTECH_MG_COMMAND_VELOCITY_CONTROL  | rxbuf[0] == LKTECH_MG_COMMAND_POSITION_CONTROL1 | \
					rxbuf[0] == LKTECH_MG_COMMAND_POSITION_CONTROL2 | rxbuf[0] == LKTECH_MG_COMMAND_POSITION_CONTROL5 | \
					rxbuf[0] == LKTECH_MG_COMMAND_POSITION_CONTROL6 )
      {
        hmotor->temperature.f = (float)((int8_t)rxbuf[1]);
        hmotor->currentRaw.b8[0] = rxbuf[2];
        hmotor->currentRaw.b8[1] = rxbuf[3];
        hmotor->current.f = ((float)hmotor->currentRaw.b16) *33.0f / 2048.0f;
        hmotor->speedRawDeg.b8[0] = rxbuf[4];
        hmotor->speedRawDeg.b8[1] = rxbuf[5];
        hmotor->speedDeg.f = ((float)hmotor->speedRawDeg.b16) / hmotor->gearRatio;
        hmotor->encoderRaw.b8[0] = rxbuf[6];
        hmotor->encoderRaw.b8[1] = rxbuf[7];
				hmotor->angle.f = ((float)hmotor->encoderRaw.b16) * 360.0f / 16383.0f / hmotor->gearRatio;
      }
      break;
      case LETECH_MG_CAN_BUS_TASK_READ_CONDITION3:
      if (rxbuf[0] == LKTECH_MG_COMMAND_READ_CONDITION3)
      {
        hmotor->temperature.f = (float)((int8_t)rxbuf[1]);
      }
      break;
  	default:
  		break;
  }
}

void LKTECH_MG_SetPIDtoRAM(LKTECH_MG_Handle* hmotor, uint8_t position_kp, uint8_t position_ki, \
                                                     uint8_t velocity_kp, uint8_t velocity_ki, \
                                                     uint8_t current_kp, uint8_t current_ki)
{
  hmotor->task = LKTECH_MG_CAN_BUS_TASK_SET_PID_TO_RAM;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_SET_PID_TO_ROM;
  hmotor->txBuf[2] = position_kp;
  hmotor->txBuf[3] = position_ki;
  hmotor->txBuf[4] = velocity_kp;
  hmotor->txBuf[5] = velocity_ki;
  hmotor->txBuf[6] = current_kp;
  hmotor->txBuf[7] = current_ki;
}

void LKTECH_MG_ReadPID(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_PID, LKTECH_MG_CAN_BUS_TASK_READ_PID);
}

void LKTECH_MG_ReadAcceleration(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_ACCELERATION, LKTECH_MG_CAN_BUS_TASK_READ_ACCELERATION);
}

void LETECH_MG_SetAccelerationToRAM(LKTECH_MG_Handle* hmotor, int32_t acceleration_deg)
{
  hmotor->task = LETECH_MG_CAN_BUS_TASK_SET_ACCELERATION_TO_RAM;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_SET_ACCELERATION_TO_RAM;
  union Int32UInt8 acc;
  acc.b32 = acceleration_deg;
  hmotor->txBuf[4] = acc.b8[0];
  hmotor->txBuf[5] = acc.b8[1];
  hmotor->txBuf[6] = acc.b8[2];
  hmotor->txBuf[7] = acc.b8[3];
}

void LETECH_MG_ZeroingByCurrentPosition(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_ZEROING_BY_CURRENT_POSITION, LETECH_MG_CAN_BUS_TASK_ZEROING_BY_CURRENT_POSITION);
}

void LETECH_MG_ReadAngleSingleTurn(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_ANGLE_SINGLE_TURN, LETECH_MG_CAN_BUS_TASK_READ_ANGLE_SINGLE_TURN);
}

void LETECH_MG_ReadAngleMultiTurn(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_ANGLE_MULTI_TURN, LETECH_MG_CAN_BUS_TASK_READ_ANGLE_MULTI_TURN);
}

void LETECH_MG_ReadCondition1andError(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_CONDITION1_N_ERROR, LETECH_MG_CAN_BUS_TASK_READ_CONDITION1_AND_ERROR);
}
void LETECH_MG_ReadCondition2(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_CONDITION2, LETECH_MG_CAN_BUS_TASK_READ_CONDITION2);
}
void LETECH_MG_ReadCondition3(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_READ_CONDITION3, LETECH_MG_CAN_BUS_TASK_READ_CONDITION3);
}

void LETECH_MG_Shutdown(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_SHUT_DOWN, LETECH_MG_CAN_BUS_TASK_SHUTDOWN);
}

void LETECH_MG_Disable(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_DISABLE, LETECH_MG_CAN_BUS_TASK_DISABLE);
}

void LETECH_MG_Enable(LKTECH_MG_Handle* hmotor)
{
  LKTECH_MG_SendSingleCommand(hmotor, LKTECH_MG_COMMAND_ENABLE, LETECH_MG_CAN_BUS_TASK_ENABLE);
}

void LETECH_MG_CurrentControl(LKTECH_MG_Handle* hmotor, float iq)
{
  hmotor->task = LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_CURRENT_CONTROL;
  union Int16UInt8 current;
  current.b16 = (int16_t)(iq * 2000.0f / 32.0f);
  hmotor->txBuf[4] = current.b8[0];
  hmotor->txBuf[5] = current.b8[1];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void LETECH_MG_SpeedControl(LKTECH_MG_Handle* hmotor, float spd)
{
  hmotor->task = LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_VELOCITY_CONTROL;
  union Int32UInt8 vel;
  vel.b32 = (int32_t)(spd * 100.0f * hmotor->gearRatio);
  hmotor->txBuf[4] = vel.b8[0];
  hmotor->txBuf[5] = vel.b8[1];
  hmotor->txBuf[6] = vel.b8[2];
  hmotor->txBuf[7] = vel.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void LETECH_MG_PositionControl1MultiTurn(LKTECH_MG_Handle* hmotor, float pos_multi)
{
  hmotor->positionControlMultiTurnSet.f = pos_multi;
  union Int32UInt8 pos;
  pos.b32 = (int32_t)(pos_multi * 100.0f);
  hmotor->task = LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_1_MULTI_TURN;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_POSITION_CONTROL1;
  hmotor->txBuf[4] = pos.b8[0];
  hmotor->txBuf[5] = pos.b8[1];
  hmotor->txBuf[6] = pos.b8[2];
  hmotor->txBuf[7] = pos.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void LETECH_MG_PositionControl2MultiTurn(LKTECH_MG_Handle* hmotor, float pos_multi, float vel_limit)
{
  hmotor->positionControlMultiTurnSet.f = pos_multi;
  union Int32UInt8 pos;
  union UInt16UInt8 vel;
  vel.b16 = (uint16_t)vel_limit;
  pos.b32 = (int32_t)(pos_multi * 100.0f);
  hmotor->task = LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_2_MULTI_TURN;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_POSITION_CONTROL2;
  hmotor->txBuf[2] = vel.b8[0];
  hmotor->txBuf[3] = vel.b8[1];
  hmotor->txBuf[4] = pos.b8[0];
  hmotor->txBuf[5] = pos.b8[1];
  hmotor->txBuf[6] = pos.b8[2];
  hmotor->txBuf[7] = pos.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void LETECH_MG_PositionControl5Increment(LKTECH_MG_Handle* hmotor, float pos_incre)
{
  hmotor->positionControlMultiTurnSet.f = pos_incre;
  union Int32UInt8 pos;
  pos.b32 = (int32_t)(pos_incre * 100.0f);
  hmotor->task = LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_5_INCREMENT;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_POSITION_CONTROL5;
  hmotor->txBuf[4] = pos.b8[0];
  hmotor->txBuf[5] = pos.b8[1];
  hmotor->txBuf[6] = pos.b8[2];
  hmotor->txBuf[7] = pos.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}
void LETECH_MG_PositionControl6Increment(LKTECH_MG_Handle* hmotor, float pos_incre, float vel_limit)
{
  hmotor->positionControlMultiTurnSet.f = pos_incre;
  union Int32UInt8 pos;
  union UInt16UInt8 vel;
  vel.b16 = (uint16_t)vel_limit;
  pos.b32 = (int32_t)(pos_incre * 100.0f);
  hmotor->task = LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT;
  memset(hmotor->txBuf, 0, 8);
  hmotor->txBuf[0] = LKTECH_MG_COMMAND_POSITION_CONTROL6;
  hmotor->txBuf[2] = vel.b8[0];
  hmotor->txBuf[3] = vel.b8[1];
  hmotor->txBuf[4] = pos.b8[0];
  hmotor->txBuf[5] = pos.b8[1];
  hmotor->txBuf[6] = pos.b8[2];
  hmotor->txBuf[7] = pos.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}
