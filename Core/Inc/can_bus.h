#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"

#define CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_THIGH      21U
#define CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_THIGH      22U
#define CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_THIGH      23U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH      24U
#define CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_SHANK      25U
#define CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_SHANK      26U
#define CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_SHANK      27U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK      28U
#define CAN_ID_IMU_TORSO_ANGLE_EXOSKELETON             29U

#define CAN_ID_TMOTOR_RX      											            0x00
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_MOTOR              0x07
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_MASTER             0x08
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_MOTOR               0x09
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_MASTER              0x0A
#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_KNEE_MOTOR              0x03
#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_KNEE_MASTER             0x04
#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP_MOTOR               0x05
#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP_MASTER              0x06


CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2);
void ConfigCANFilter_STD_ID_16Bit4IDListMode(CAN_HandleTypeDef* hcan,  uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4);
void ConfigCANFilter_STD_ID_32Bit2IDListMode(CAN_HandleTypeDef* hcan,  uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t ID1, uint8_t ID2);

void CAN_ConfigureFilters(void);

#endif
