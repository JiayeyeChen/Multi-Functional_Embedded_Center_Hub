#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"

#define CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_THIGH       0x01
#define CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_THIGH        0x02
#define CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_THIGH  0x03
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH      0x04
#define CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_THIGH         0x05
#define CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_THIGH         0x06
#define CAN_ID_IMU_GET_DATA_NDOF_RIGHT_THIGH           0x07
#define CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_THIGH       0x08
#define CAN_ID_IMU_GET_DATA_AMG_RIGHT_THIGH            0x09

#define CAN_ID_IMU_LIACC_EXOSKELETON_RIGHT_SHANK       0x0B
#define CAN_ID_IMU_GYRO_EXOSKELETON_RIGHT_SHANK        0x0C
#define CAN_ID_IMU_QUATERNION_EXOSKELETON_RIGHT_SHANK  0x0D
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK      0x0E
#define CAN_ID_IMU_ACC_EXOSKELETON_RIGHT_SHANK         0x0F
#define CAN_ID_IMU_MAG_EXOSKELETON_RIGHT_SHANK         0x10
#define CAN_ID_IMU_GET_DATA_NDOF_RIGHT_SHANK           0x11
#define CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_SHANK       0x12
#define CAN_ID_IMU_GET_DATA_AMG_RIGHT_SHANK            0x13

#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP           0x2904
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP          0x2901
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE         0x2902

CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2);

void CAN_ConfigureFilters(void);

#endif
