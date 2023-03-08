#ifndef MRDOOR_H
#define MRDOOR_H

#include "system_periphrals.h"
#include "tmotor_ak10-9_v2.h"
#include "my_math.h"
#include <math.h>

void MRDOOR_MotorInit(void);
void MRDOOR_CalculateWeight(AK10_9HandleCubaMarsFW* hmotor_left, \
                            AK10_9HandleCubaMarsFW* hmotor_right, \
                            float k_left, float k_right, float b_left, float b_right);




extern float MrDoorControllingSupportWeight;
extern float MrDoorRealWeightLeft, MrDoorRealWeightRight;
extern AK10_9HandleCubaMarsFW hAKMotorMrDoorLeft, hAKMotorMrDoorRight;

#endif
