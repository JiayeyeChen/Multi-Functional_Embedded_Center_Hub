#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"

#define CAN_ID_ENCODER_RX_DATA            0x01
#define CAN_ID_ENCODER_RIGHT_TURN 	      0x01
#define CAN_ID_ENCODER_LEFT_TURN		      0x02
#define CAN_ID_ENCODER_RIGHT_PULL		      0x03
#define CAN_ID_ENCODER_LEFT_PULL		      0x04
#define CAN_ID_ENCODER_RIGHT_WHEEL        0x05
#define CAN_ID_ENCODER_LEFT_WHEEL         0x06
#define CAN_ID_ENCODER_UPPER_LIMB_2       0x08
#define CAN_ID_ENCODER_UPPER_LIMB_1       0x22
#define CAN_ID_AK10_9_DMFW_M1_RX                       0x0A
#define CAN_ID_AK10_9_DMFW_M2_RX                       0x0B
#define CAN_ID_AK10_9_DMFW_M3_RX                       0x0C
#define CAN_ID_UPPER_LIMB_FORCE_SENSOR_TX              0x05
#define CAN_ID_UPPER_LIMB_FORCE_SENSOR_RX              0x06

#define CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_THIGH      21U
#define CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_THIGH      22U
#define CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_THIGH      23U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_THIGH      24U
#define CAN_ID_IMU_X_DATA_EXOSKELETON_RIGHT_SHANK      25U
#define CAN_ID_IMU_Y_DATA_EXOSKELETON_RIGHT_SHANK      26U
#define CAN_ID_IMU_Z_DATA_EXOSKELETON_RIGHT_SHANK      27U
#define CAN_ID_IMU_STATUS_EXOSKELETON_RIGHT_SHANK      28U
#define CAN_ID_IMU_TORSO_ANGLE_EXOSKELETON             29U

#define CAN_ID_TMOTOR_SPARE1_SERVOMODE                 0x2903
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_SERVO_MODE          0x2901//0x2904
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_TX_SERVO_MODE         0x2902//0x2902
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_RX                  0x0D
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_TX                  0x04
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_RX                 0x00
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_TX                 0x02

CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2);
void ConfigCANFilter_STD_ID_16Bit4IDListMode(CAN_HandleTypeDef* hcan,  uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4);


void CAN_ConfigureFilters(void);

#endif
