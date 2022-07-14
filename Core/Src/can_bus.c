#include "can_bus.h"
#include "ak10-9_v2_testing.h"
#include "usb.h"
#include "exoskeleton.h"
#include "encoder.h"

//for testing//
uint32_t rxfifo0detected = 0;
uint32_t rxfifo1detected = 0;
uint32_t wheelcount_left = 0, wheelcountsec_left = 0;
uint32_t wheelcount_right = 0, wheelcountsec_right = 0;
uint32_t armcount = 0, armcountsec = 0;
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
  //Filter bank 0 & 1
  /*Servo mode*/
//  hAKMotorRightKnee.rxFilter = ConfigCANFilter_EXT_ID_32BitIDListMode(&hcan2, 0, CAN_FILTER_FIFO1, CAN_ID_EXT, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_TX_SERVO_MODE, 0);
//  hAKMotorRightHip_old.rxFilter = ConfigCANFilter_EXT_ID_32BitIDListMode(&hcan2, 1, CAN_FILTER_FIFO1, CAN_ID_EXT, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_SERVO_MODE, 0);
  /*MIT mode*/
  hAKMotorRightHip.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hAKMotorRightHip.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hAKMotorRightHip.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	hAKMotorRightHip.rxFilter.FilterBank = 0;
	hAKMotorRightHip.rxFilter.FilterIdHigh = CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_RX << 5;
	hAKMotorRightHip.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hAKMotorRightHip.hcan, &hAKMotorRightHip.rxFilter);
  hAKMotorRightKnee.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
	hAKMotorRightKnee.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
	hAKMotorRightKnee.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO1;
	hAKMotorRightKnee.rxFilter.FilterBank = 1;
	hAKMotorRightKnee.rxFilter.FilterIdHigh = hAKMotorRightKnee.canID << 5;
	hAKMotorRightKnee.rxFilter.FilterActivation = ENABLE;
	HAL_CAN_ConfigFilter(hAKMotorRightKnee.hcan, &hAKMotorRightKnee.rxFilter);
//  //Filter bank 2
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 2;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 3
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 3;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 4
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 4;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 5
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 5;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 6
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 6;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 7
//  hIMURightThigh.rxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//	hIMURightThigh.rxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//	hIMURightThigh.rxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//	hIMURightThigh.rxFilter.FilterBank = 7;
//	hIMURightThigh.rxFilter.FilterIdHigh = CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_THIGH << 5;
//	hIMURightThigh.rxFilter.FilterActivation = ENABLE;
//	HAL_CAN_ConfigFilter(hIMURightThigh.hcan, &hIMURightThigh.rxFilter);
//  //Filter bank 8
//  hEncoderRightWheel.canRxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//  hEncoderRightWheel.canRxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//  hEncoderRightWheel.canRxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  hEncoderRightWheel.canRxFilter.FilterBank = 8;
//  hEncoderRightWheel.canRxFilter.FilterIdHigh = CAN_ID_ENCODER_RIGHT_WHEEL << 5;
//  hEncoderRightWheel.canRxFilter.FilterActivation = ENABLE;
//  HAL_CAN_ConfigFilter(hEncoderRightWheel.hcan, &hEncoderRightWheel.canRxFilter);
//  //Filter bank 9
//  hEncoderLeftWheel.canRxFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//  hEncoderLeftWheel.canRxFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//  hEncoderLeftWheel.canRxFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  hEncoderLeftWheel.canRxFilter.FilterBank = 9;
//  hEncoderLeftWheel.canRxFilter.FilterIdHigh = CAN_ID_ENCODER_LEFT_WHEEL << 5;//
//  hEncoderLeftWheel.canRxFilter.FilterActivation = ENABLE;
//  HAL_CAN_ConfigFilter(hEncoderLeftWheel.hcan, &hEncoderLeftWheel.canRxFilter);
//  //Filter bank 10
//  CAN_FilterTypeDef hArmEncodersFilter;
//  hArmEncodersFilter.FilterMode = CAN_FILTERMODE_IDLIST;
//  hArmEncodersFilter.FilterScale = CAN_FILTERSCALE_16BIT;
//  hArmEncodersFilter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
//  hArmEncodersFilter.FilterBank = 10;
//  hArmEncodersFilter.FilterIdHigh = CAN_ID_ENCODER_RX_DATA << 5;
//  hArmEncodersFilter.FilterActivation = ENABLE;
//  HAL_CAN_ConfigFilter(&hcan2, &hArmEncodersFilter);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  //General codes
  CAN_RxHeaderTypeDef temRxHeader;
  uint8_t temRxData[8];
  HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &temRxHeader, temRxData);
  
  //Application specific codes
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
  else if (temRxHeader.StdId == CAN_ID_ENCODER_RIGHT_WHEEL)
  {
    ENCODER_GetAngle(&hEncoderRightWheel, temRxData);
    ENCODER_CalculateSpeed(&hEncoderRightWheel, 0.001f);
    wheelcount_right++;
    if (wheelcount_right >= 199)
    {
      wheelcount_right = 0;
      wheelcountsec_right++;
    }
  }
  else if (temRxHeader.StdId == CAN_ID_ENCODER_LEFT_WHEEL)
  {
    ENCODER_GetAngle(&hEncoderLeftWheel, temRxData);
//    ENCODER_CalculateSpeed(&hEncoderLeftWheel, 0.001f);
    wheelcount_left++;
    if (wheelcount_left >= 199)
    {
      wheelcount_left = 0;
      wheelcountsec_left++;
    }
  }
  
  if (temRxHeader.StdId == CAN_ID_ENCODER_RX_DATA)
  {
    ENCODER_GetAngle(hEncoderPtr, temRxData);
    ENCODER_CalculateSpeed(hEncoderPtr, 0.001f);
    armcount++;
    if (armcount >= 199)
    {
      armcount = 0;
      armcountsec++;
    }
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
  
  //Application specific codes
  if (temRxHeader.StdId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_RX)
    AK10_9_DMFW_GetFeedbackMsg(&temRxHeader, &hAKMotorRightHip, temRxData);
  else if (temRxHeader.StdId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_RX)
    AK10_9_MITMode_GetFeedbackMsg(&temRxHeader, &hAKMotorRightKnee, temRxData);
  
  /*Exoskeleton tmotor servo mode, Briter encoders and Tmotor DMFW*/
////////////////  if (temRxHeader.ExtId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_SERVO_MODE)
////////////////  {
////////////////    AK10_9_ServoMode_GetFeedbackMsg(&temRxHeader, &hAKMotorRightHip_old, temRxData);
////////////////  }
////////////////  else if (temRxHeader.ExtId == CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_TX_SERVO_MODE)
////////////////  {
////////////////    AK10_9_ServoMode_GetFeedbackMsg(&temRxHeader, &hAKMotorRightKnee, temRxData);
////////////////  }
////////////////  else if (temRxHeader.StdId == CAN_ID_AK10_9_DMFW_M1_RX)
////////////////    AK10_9_DMFW_GetFeedbackMsg(&temRxHeader, &hAKMotorDMFW1, temRxData);
////////////////  
////////////////  if (temRxHeader.StdId == CAN_ID_ENCODER_RX_DATA)
////////////////  {
////////////////    ENCODER_GetAngle(&hEncoderLeftPull, temRxData);
////////////////    ENCODER_CalculateSpeed(&hEncoderLeftPull, 0.001f);
////////////////    armcount++;
////////////////    if (armcount >= 999)
////////////////    {
////////////////      armcount = 0;
////////////////      armcountsec++;
////////////////    }
////////////////  }
  rxfifo1detected++;
}
