#include "can_bus.h"
#include "ak10-9_v2_testing.h"
#include "usb.h"
#include "exoskeleton.h"
#include "bldc_actuators_testing.h"

//for testing//
uint32_t rxfifo0detected = 0;
uint32_t rxfifo1detected = 0;
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

void ConfigCANFilter_STD_ID_16Bit4IDListMode(CAN_HandleTypeDef* hcan,  uint32_t FilterBank, \
                                             uint32_t FilterFIFOAssignment, uint8_t ID1, uint8_t ID2, \
                                             uint8_t ID3, uint8_t ID4)
{
  CAN_FilterTypeDef filter;
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
	filter.FilterScale = CAN_FILTERSCALE_16BIT;
	filter.FilterFIFOAssignment = FilterFIFOAssignment;
	filter.FilterBank = FilterBank;
	filter.FilterIdHigh = ID1 << 5;
  filter.FilterIdLow = ID2 << 5;
  filter.FilterMaskIdHigh = ID3 << 5;
  filter.FilterMaskIdLow = ID4 << 5;
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &filter);
}

void ConfigCANFilter_STD_ID_32Bit2IDListMode(CAN_HandleTypeDef* hcan,  uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t ID1, uint8_t ID2)
{
	CAN_FilterTypeDef filter;
	filter.FilterMode = CAN_FILTERMODE_IDLIST;;
	filter.FilterScale = CAN_FILTERSCALE_32BIT;
	filter.FilterFIFOAssignment = FilterFIFOAssignment;
	filter.FilterBank = FilterBank;
	filter.FilterIdHigh = ID1 << 5;
	filter.FilterIdLow = ID2 << 5;
	filter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hcan, &filter);
}

void CAN_ConfigureFilters(void)
{
  CAN_FilterTypeDef tempFilter;

  /*Filter bank 0 & 1*/
  /*******************/
	tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;;
	tempFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	tempFilter.FilterBank = 0;
	tempFilter.FilterIdHigh = CAN_ID_TMOTOR_RX << 5;
	tempFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(&hcan2, &tempFilter);
  /*Filter bank 2*/

  /***************/

  /*Filter bank 3*/
	
	tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;;
	tempFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	tempFilter.FilterBank = 3;
	tempFilter.FilterIdHigh = hLKTECH.canID << 5;
	tempFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hLKTECH.hcan, &tempFilter);
  /***************/
  
  /*Filter bank 4*/
  tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;;
	tempFilter.FilterScale = CAN_FILTERSCALE_32BIT;
	tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	tempFilter.FilterBank = 4;
	tempFilter.FilterIdHigh = hLKTECH.canID << 5;
	tempFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hLKTECH.hcan, &tempFilter);
  /***************/

  /*Filter bank 5*/
  /***************/
//  hIMUTorso.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMUTorso.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMUTorso.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMUTorso.rxFilter.FilterBank = 5;
//	hIMUTorso.rxFilter.FilterIdHigh = CAN_ID_IMU_TORSO_ANGLE_EXOSKELETON << 5;
//	hIMUTorso.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMUTorso.hcan, &hIMUTorso.rxFilter);
  /*Filter bank 6*/
  /***************/
//  tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	tempFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	tempFilter.FilterBank = 6;
//	tempFilter.FilterIdHigh = CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_THIGH << 5;
//	tempFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMUHip.hcan, &tempFilter);
  /*Filter bank 7*/
  /***************/
//  ConfigCANFilter_STD_ID_16Bit4IDListMode(hIMUHip.hcan, 7, CAN_FILTER_FIFO0, \
//                                          CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_THIGH, \
//                                          CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_THIGH, \
//                                          CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_THIGH, \
//                                          CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH);
  /*Filter bank 8*/
  /***************/
//  ConfigCANFilter_STD_ID_16Bit4IDListMode(hIMUKnee.hcan, 8, CAN_FILTER_FIFO0, \
//                                          CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_SHANK, \
//                                          CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_SHANK, \
//                                          CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_SHANK, \
//                                          CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK);
  /*Filter bank 9*/
  /***************/
//  tempFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//  tempFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//  tempFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  tempFilter.FilterBank = 9;
//  tempFilter.FilterIdHigh = CAN_ID_ENCODER_UPPER_LIMB_1 << 5;
//  tempFilter.FilterActivation = ENABLE;
//  HAL_CAN_ConfigFilter(&hcan2, &tempFilter);
  /*Filter bank 10*/
  /***************/
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //General codes
  CAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &temRxHeader, temRxData);
  
  //Application specific codes
  if (temRxHeader.StdId == CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetJointAccelerationIMUFeedback_X_Data(&hIMUHip, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetJointAccelerationIMUFeedback_Y_Data(&hIMUHip, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetJointAccelerationIMUFeedback_Z_Data(&hIMUHip, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH)
    EXOSKELETON_GetJointAccelerationIMUFeedback_BNO055Status(&hIMUHip, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_SHANK)
    EXOSKELETON_GetJointAccelerationIMUFeedback_X_Data(&hIMUKnee, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_SHANK)
    EXOSKELETON_GetJointAccelerationIMUFeedback_Y_Data(&hIMUKnee, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_SHANK)
    EXOSKELETON_GetJointAccelerationIMUFeedback_Z_Data(&hIMUKnee, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK)
    EXOSKELETON_GetJointAccelerationIMUFeedback_BNO055Status(&hIMUKnee, temRxData);
  else if (temRxHeader.StdId == CAN_ID_IMU_TORSO_ANGLE_EXOSKELETON)
    EXOSKELETON_GetBNO055FeedbackGrv(&hIMUTorso, temRxData);
	
  if (temRxHeader.StdId == hLKTECH.canID)
    LKTECH_MG_GetFeedback(&hLKTECH, &temRxHeader, temRxData);
  
  
	if (temRxHeader.StdId == CAN_ID_TMOTOR_RX)
	{
	}
  
  //End
  rxfifo0detected++;
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //General codes
  CAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &temRxHeader, temRxData);
  
  if (temRxHeader.StdId == hLKTECH.canID)
    LKTECH_MG_GetFeedback(&hLKTECH, &temRxHeader, temRxData);
  
  
  
  //Application specific codes
  
  
  rxfifo1detected++;
}
