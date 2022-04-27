#ifndef CAN_BUS_H
#define CAN_BUS_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"


#define CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP          0x2904
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP          0x2901
#define CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE         0x2902

CAN_FilterTypeDef ConfigCANFilter_EXT_ID_32BitIDListMode(CAN_HandleTypeDef* hcan, uint32_t FilterBank, uint32_t FilterFIFOAssignment, uint8_t IDE, uint32_t ID1, uint32_t ID2);

void CAN_ConfigureFilter(void);







#endif
