#include "can_bus.h"
#include "ak10-9_v2_testing.h"
#include "usb.h"
#include "exoskeleton.h"

//for testing//
uint8_t rxfifo0detected = 0;
uint8_t rxfifo1detected = 0;

///////////////
CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2)
{
  CAN_FilterTypeDef filter;
  filter.FilterBank = FilterBank;
  filter.FilterFIFOAssignment = FilterFIFOAssignment;
  
  if (IDE == CAN_ID_STD)
  {
    filter.FilterIdHigh = ID1 << 5;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = ID1 << 5;
    filter.FilterMaskIdLow = 0;
  }
  else if (IDE == CAN_ID_EXT)
  {
    filter.FilterIdHigh = (ID1 >> 13) & 0xFFFF;
    filter.FilterIdLow = ((ID1 & 0x1FFF) << 3) | 4;
    filter.FilterMaskIdHigh = (ID2 >> 13) & 0xFFFF;
    filter.FilterMaskIdLow = ((ID2 & 0x1FFF) << 3) | 4;
  }
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan, &filter);
  
  return filter;
}

void CAN_ConfigureFilters(void)
{
  //Filter bank 0
  hAKMotorRightKnee.rxFilter = ConfigCANFilter_EXT_ID_32BitIDListMode(&hcan2, 0, CAN_FILTER_FIFO1, CAN_ID_EXT, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE, 0);
  //Filter bank 1
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 1;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
  //Filter bank 2
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 2;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
  //Filter bank 3
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 5;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
  //Filter bank 4
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 4;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
  //Filter bank 5
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 3;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
  //Filter bank 6
  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	hIMURightThigh.rxFilter.FilterBank = 6;
	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_THIGH << 5;
	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //General codes
  CAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &temRxHeader, temRxData);
  
  //Application specific codes
  //BNO055
  if (temRxHeader.StdId == CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackLiAcc(&hIMURightThigh, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackGyro(&hIMURightThigh, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackQuaternion(&hIMURightThigh, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackStatus(&hIMURightThigh, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackAcc(&hIMURightThigh, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetIMUFeedbackMag(&hIMURightThigh, temRxData);
  //End
  rxfifo0detected++;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //General codes
  CAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &temRxHeader, temRxData);
  
  //Application specific codes
  //Tmotors
  if (temRxHeader.ExtId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP)
  {
    AK10_9_ServoMode_GetFeedbackMsg(&temRxHeader, &hAKMotorRightHip, temRxData);
    AK10_9_Calculate_velocity_current_AVG(&hAKMotorRightHip);
  }
  else if (temRxHeader.ExtId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE)
  {
    AK10_9_ServoMode_GetFeedbackMsg(&temRxHeader, &hAKMotorRightKnee, temRxData);
    AK10_9_Calculate_velocity_current_AVG(&hAKMotorRightKnee);
  }
  rxfifo1detected++;
}
