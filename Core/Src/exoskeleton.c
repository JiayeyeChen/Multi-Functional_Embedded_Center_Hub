#include "exoskeleton.h"

BNO055Handle hIMURightThigh, hIMURightKnee;

void EXOSKELETON_Init(void)
{
  hIMURightThigh.hcan = &hcan2;
  hIMURightThigh.operationMode = 0xFF;
  hIMURightThigh.operationModeENUM = IMU_MODE_ACCONLY;
  hIMURightThigh.CANID_SET_MODE_NDOF = CAN_ID_IMU_GET_DATA_NDOF_RIGHT_THIGH;
  hIMURightThigh.CANID_SET_MODE_GYROONLY = CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_THIGH;
  hIMURightThigh.CANID_SET_MODE_ACCONLY = CAN_ID_IMU_GET_DATA_ACCONLY_RIGHT_THIGH;
}

void EXOSKELETON_GetIMUFeedbackLiAcc(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.liaccX.b8[0] = data[0];
  himu->rawData.liaccX.b8[1] = data[1];
  himu->rawData.liaccY.b8[0] = data[2];
  himu->rawData.liaccY.b8[1] = data[3];
  himu->rawData.liaccZ.b8[0] = data[4];
  himu->rawData.liaccZ.b8[1] = data[5];
}

void EXOSKELETON_GetIMUFeedbackGyro(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.gyroX.b8[0] = data[0];
  himu->rawData.gyroX.b8[1] = data[1];
  himu->rawData.gyroY.b8[0] = data[2];
  himu->rawData.gyroY.b8[1] = data[3];
  himu->rawData.gyroZ.b8[0] = data[4];
  himu->rawData.gyroZ.b8[1] = data[5];
}
void EXOSKELETON_GetIMUFeedbackQuaternion(BNO055Handle* himu, uint8_t data[])
{
  
}

void EXOSKELETON_GetIMUFeedbackStatus(BNO055Handle* himu, uint8_t data[])
{
  himu->calibStatus = data[0];
  himu->operationMode = data[1];
  himu->deadCount = data[2];
}

void EXOSKELETON_SetIMUMode_9_DOF(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_NDOF;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}
void EXOSKELETON_SetIMUMode_GYRO_Only(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_GYROONLY;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}

void EXOSKELETON_SetIMUMode_ACC_Only(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_ACCONLY;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}

void EXOSKELETON_GetIMUFeedbackAcc(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.AccX.b8[0] = data[0];
  himu->rawData.AccX.b8[1] = data[1];
  himu->rawData.AccY.b8[0] = data[2];
  himu->rawData.AccY.b8[1] = data[3];
  himu->rawData.AccZ.b8[0] = data[4];
  himu->rawData.AccZ.b8[1] = data[5];
}
void EXOSKELETON_GetIMUFeedbackMag(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.MagX.b8[0] = data[0];
  himu->rawData.MagX.b8[1] = data[1];
  himu->rawData.MagY.b8[0] = data[2];
  himu->rawData.MagY.b8[1] = data[3];
  himu->rawData.MagZ.b8[0] = data[4];
  himu->rawData.MagZ.b8[1] = data[5];
}
