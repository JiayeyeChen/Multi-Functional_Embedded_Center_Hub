#ifndef FOSHAN_4DOF_EXOSKELETON_TMOTOR_H
#define FOSHAN_4DOF_EXOSKELETON_TMOTOR_H

#include "tmotor_ak10-9_v2.h"
#include "system_periphrals.h"

typedef struct
{
	AK10_9Handle hMotorLeftHip, hMotorLeftKnee, hMotorRightHip, hMotorRightKnee;
	float controlLoopPeriod;
	
}Foshan4DOFExoskeletonTMotorHandle;

void Foshan4DOFExoskeletonTMotor_Init(float controlLoopPeriod);

void Foshan4DOFExoskeletonTMotor_CenterControl(void);






extern Foshan4DOFExoskeletonTMotorHandle hExoskeletonFoshan4DOFTMotor;

#endif
