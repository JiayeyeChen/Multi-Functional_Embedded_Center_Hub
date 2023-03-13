#ifndef MRDOOR_H
#define MRDOOR_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"
#include "my_math.h"
#include <math.h>
#include "BenMoKeJi_M15.h"

void MRDOOR_MotorInit(void);
void MRDOOR_CalculateWeightTMotor(AK10_9HandleCubaMarsFW* hmotor_left, \
                            AK10_9HandleCubaMarsFW* hmotor_right, \
                            float k_left, float k_right, float b_left, float b_right);
void MRDOOR_CalculateWeightBenMoKeJi(BENMOKEJI_M15_Handle* hmotor_left, \
                                     BENMOKEJI_M15_Handle* hmotor_right, \
                                     float k_left, float k_right, float b_left, float b_right);



extern float MrDoorControllingSupportWeight;
extern float MrDoorRealWeightLeft, MrDoorRealWeightRight;
extern BENMOKEJI_M15_Handle   hBENMOKEJIMrDoorLeft, hBENMOKEJIMrDoorRight;
extern AK10_9HandleCubaMarsFW hAKMotorMrDoorLeft, hAKMotorMrDoorRight;

#endif
