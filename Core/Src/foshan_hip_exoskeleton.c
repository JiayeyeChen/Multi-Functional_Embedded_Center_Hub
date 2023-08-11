#include "foshan_hip_exoskeleton.h"

FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

void FOSHANHIPEXOSKELETON_Init(void)
{
  LKTECH_MG_Init(&hFoshanHipExoskeleton.hMotorLeft, &hcan1, 1, 10.0f, 1.8226f);
  hFoshanHipExoskeleton.task = FOSHAN_HIP_EXOSKELETON_TASK_NONE;
  hFoshanHipExoskeleton.gravityFactor = 0.0f;
  hFoshanHipExoskeleton.innertiaFactor = 0.0f;
  hFoshanHipExoskeleton.springFactor = 0.0f;
  hFoshanHipExoskeleton.leftAngleOffset = 55.0f;
}

void FOSHANHIPEXOSKELETON_CentreControl(void)
{
}
