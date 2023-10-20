#ifndef __FOSHAN_HIP_EXOSKELETON_H
#define __FOSHAN_HIP_EXOSKELETON_H

#include "xiaomi_cybergear.h"
#include "cmsis_os.h"
#include "common.h"
#include "system_periphrals.h"

enum FoshanHipExoskeletonTask
{
  FOSHAN_HIP_EXOSKELETON_TASK_NONE,
  FOSHAN_HIP_EXOSKELETON_TASK_ASSIST,
  FOSHAN_HIP_EXOSKELETON_TASK_RESIST
};

typedef struct
{
  CybergearHandle hMotorLeft, hMotorRight;
  float leftAngleOffset, rightAngleOffset;
  float leftOffsetedAngle, rightOffsetedAngle;
  float leftDirection, rightDirection;
  enum FoshanHipExoskeletonTask task;
  float       gravityFactor, innertiaFactor, springFactor;
  uint8_t switchtask;
  float assistiveTorqueLeft, resistiveTorqueLeft;
	float assistiveTorqueRight, resistiveTorqueRight;
  LowPassFilterHandle assistiveTorqueFilteredLeft, resistiveTorqueFilteredLeft;
	LowPassFilterHandle assistiveTorqueFilteredRight, resistiveTorqueFilteredRight;
}FoshanHipExoskeletonHandle;

void FOSHANHIPEXOSKELETON_Init(float loop_duration_second);
void FOSHANHIPEXOSKELETON_CentreControl(void);


extern FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

#endif
