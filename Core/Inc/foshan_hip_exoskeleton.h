#ifndef __FOSHAN_HIP_EXOSKELETON_H
#define __FOSHAN_HIP_EXOSKELETON_H

#include "lktech_mg_motor.h"

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
  enum FoshanHipExoskeletonTask task;
  float       gravityFactor, innertiaFactor, springFactor;
}FoshanHipExoskeletonHandle;

void FOSHANHIPEXOSKELETON_Init(void);
void FOSHANHIPEXOSKELETON_CentreControl(void);



extern FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

#endif
