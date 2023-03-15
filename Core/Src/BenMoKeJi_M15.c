#include "BenMoKeJi_M15.h"

void BENMOKEJI_M15_Init(BENMOKEJI_M15_Handle* hmotor, CAN_HandleTypeDef* hcan, uint8_t motor_id)
{
	hmotor->hcan = hcan;
	hmotor->motorID = motor_id;
	hmotor->canIDFeedback = 0x96 + hmotor->motorID;
	hmotor->mode = BENMOKEJI_MODE_DISABLED;
  hmotor->errorCode = 0xFF;
}

void BENMOKEJI_M15_GetFeedback(BENMOKEJI_M15_Handle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[])
{
	hmotor->speedRaw.b8[1] = rxbuf[0];
	hmotor->speedRaw.b8[0] = rxbuf[1];
	hmotor->currentRaw.b8[1] = rxbuf[2];
	hmotor->currentRaw.b8[0] = rxbuf[3];
	hmotor->positionRaw.b8[1] = rxbuf[4];
	hmotor->positionRaw.b8[0] = rxbuf[5];
  hmotor->errorCode = rxbuf[6];
	
	hmotor->speedRealDeg.f = ((float)hmotor->speedRaw.b16) * 6.0f;
	hmotor->currentReal.f = ((float)hmotor->currentRaw.b16) * 33.0f / 32767.0f;
	hmotor->positionRealDeg.f = ((float)hmotor->positionRaw.b16) * 360.0f / 32767.0f;
}

void BENMOKEJI_M15_SetFeedbackMode(BENMOKEJI_M15_Handle* hmotor, uint8_t mode, uint8_t auto_feedback_duration)
{
	hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
	hmotor->txHeader.StdId = 0x106;
	if (mode == BENMOKEJI_M15_FEEDBACK_MODE_AUTO)
		hmotor->txBuf[hmotor->motorID - 1] = auto_feedback_duration;
	else if (mode == BENMOKEJI_M15_FEEDBACK_MODE_MANUAL)
		hmotor->txBuf[hmotor->motorID - 1] = 0x80;
	
	HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void BENMOKEJI_M15_SendControlValueSingleMotor(BENMOKEJI_M15_Handle* hmotor, int16_t val)
{
	hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
	
	if (hmotor->motorID <= 4)
	{
		hmotor->txHeader.StdId = 0x32;
		hmotor->txBuf[hmotor->motorID * 2 - 2] = (uint8_t)(0x00FF & (val >> 8));
		hmotor->txBuf[hmotor->motorID * 2 - 1] = (uint8_t)(0x00FF & val);
	}
	else if (hmotor->motorID >= 5)
	{
		hmotor->txHeader.StdId = 0x33;
		hmotor->txBuf[(hmotor->motorID - 4) * 2 - 2] = (uint8_t)(0x00FF & (val >> 8));
		hmotor->txBuf[(hmotor->motorID - 4) * 2 - 1] = (uint8_t)(0x00FF & val);
	}
	
	HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void BENMOKEJI_M15_SetMode(BENMOKEJI_M15_Handle* hmotor, uint8_t mode)
{
	hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
	hmotor->txHeader.StdId = 0x105;
	memset(hmotor->txBuf, 0xFF, 8);
	hmotor->txBuf[hmotor->motorID - 1] = mode;
	HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void BENMODEJI_M15_PositionControlSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val)
{
	int16_t int16val = (int16_t)(val * 32767.0f / 360.0f);
	BENMOKEJI_M15_SendControlValueSingleMotor(hmotor, int16val);
}
void BENMODEJI_M15_VelocityControlDegSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val)
{
	int16_t int16val = (int16_t)(val / 6.0f);
	BENMOKEJI_M15_SendControlValueSingleMotor(hmotor, int16val);
}
void BENMODEJI_M15_CurrentControlSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val)
{
	int16_t int16val = (int16_t)(val * 32767.0f / 33.0f);
	BENMOKEJI_M15_SendControlValueSingleMotor(hmotor, int16val);
}

void BENMOKEJI_M15_SetCANID(BENMOKEJI_M15_Handle* hmotor, uint8_t new_id)
{
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
	hmotor->txHeader.StdId = 0x108;
	memset(hmotor->txBuf, 0xFF, 8);
	hmotor->txBuf[0] = new_id;
	HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void BENMOKEJI_M15_SetManualFeedbackMode(void)
{
  CAN_TxHeaderTypeDef txHeader;
  txHeader.DLC = 8;
  txHeader.IDE = 0;
  txHeader.RTR = 0;
	txHeader.StdId = 0x106;
  uint8_t txBuf[8];
	memset(txBuf, 0x80, 8);
  uint32_t mailBox;
	HAL_CAN_AddTxMessage(&hcan2, &txHeader, txBuf, &mailBox);
}
