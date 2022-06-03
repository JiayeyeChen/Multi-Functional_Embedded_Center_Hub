#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"

#define CAN_ID_ENCODER_RIGHT_WHEEL                     0x05
#define CAN_ID_ENCODER_LEFT_WHEEL                      0x06
#define CAN_ID_AK10_9_DMFW_M1_RX                       0x0A
#define CAN_ID_AK10_9_DMFW_M2_RX                       0x0B
#define CAN_ID_AK10_9_DMFW_M3_RX                       0x0C

#define CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_THIGH       21U
#define CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_THIGH        22U
#define CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_THIGH  23U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH      24U
#define CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_THIGH         25U
#define CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_THIGH         26U
#define CAN_ID_IMU_GET_DATA_NDOF_RIGHT_THIGH           27U
#define CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_THIGH       28U
#define CAN_ID_IMU_GET_DATA_ACCONLY_RIGHT_THIGH        29U

#define CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_SHANK       31U
#define CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_SHANK        32U
#define CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_SHANK  33U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK      34U
#define CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_SHANK         35U
#define CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_SHANK         36U
#define CAN_ID_IMU_GET_DATA_NDOF_RIGHT_SHANK           37U
#define CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_SHANK       38U
#define CAN_ID_IMU_GET_DATA_ACCONLY_RIGHT_SHANK        39U

#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP           0x2904
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP          0x2902//0x2901
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE         0x2902

CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2);

void CAN_ConfigureFilters(void);

#endif
