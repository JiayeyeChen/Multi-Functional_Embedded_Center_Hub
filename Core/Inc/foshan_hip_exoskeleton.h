#ifndef __FOSHAN_HIP_EXOSKELETON_H
#define __FOSHAN_HIP_EXOSKELETON_H

#include "lktech_mg_motor.h"
#include "cmsis_os.h"
#include "common.h"

enum FoshanHipExoskeletonTask
{
  FOSHAN_HIP_EXOSKELETON_TASK_NONE,
  FOSHAN_HIP_EXOSKELETON_TASK_ASSIST,
  FOSHAN_HIP_EXOSKELETON_TASK_RESIST
};

typedef struct
{
  LKTECH_MG_Handle hMotorLeft, hMotorRight;
  float leftAngleOffset, rightAngleOffset;
  float leftOffsetedAngle, rightOffsetedAngle;
  float leftDirection, rightDirection;
  enum FoshanHipExoskeletonTask task;
  float       gravityFactor, innertiaFactor, springFactor;
  uint8_t switchtask;
  float assistiveTorqueLeft, resistiveTorqueLeft;
  LowPassFilterHandle assistiveTorqueFilteredLeft, resistiveTorqueFilteredLeft;
}FoshanHipExoskeletonHandle;

void FOSHANHIPEXOSKELETON_Init(float loop_duration_second);
void FOSHANHIPEXOSKELETON_CentreControl(void);


extern FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

#endif
