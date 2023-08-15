#include "foshan_hip_exoskeleton.h"
FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

void FOSHANHIPEXOSKELETON_Init(float loop_duration_second)
{
  LKTECH_MG_Init(&hFoshanHipExoskeleton.hMotorLeft, &hcan2, 1, 10.0f, 1.8226f);
  hFoshanHipExoskeleton.task = FOSHAN_HIP_EXOSKELETON_TASK_NONE;
  hFoshanHipExoskeleton.gravityFactor = 0.0f;
  hFoshanHipExoskeleton.innertiaFactor = 0.0f;
  hFoshanHipExoskeleton.springFactor = 0.0f;
  hFoshanHipExoskeleton.leftAngleOffset = 60.0f;
  hFoshanHipExoskeleton.leftDirection = -1.0f;
  hFoshanHipExoskeleton.switchtask = 0;
  LowPassFilter_Init(&hFoshanHipExoskeleton.assistiveTorqueFilteredLeft, 2.0f, loop_duration_second);
  LowPassFilter_Init(&hFoshanHipExoskeleton.resistiveTorqueFilteredLeft, 2.0f, loop_duration_second);
}

void FOSHANHIPEXOSKELETON_CentreControl(void)
{
  hFoshanHipExoskeleton.leftOffsetedAngle = hFoshanHipExoskeleton.leftDirection * \
                                            (hFoshanHipExoskeleton.hMotorLeft.angle.f - \
                                            hFoshanHipExoskeleton.leftAngleOffset);
  if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_NONE)
    LETECH_MG_ReadAngleSingleTurn(&(hFoshanHipExoskeleton.hMotorLeft));
  else if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_ASSIST)
  {
    if (hFoshanHipExoskeleton.switchtask == 0)
    {
      hFoshanHipExoskeleton.assistiveTorqueLeft = hFoshanHipExoskeleton.gravityFactor * sinf(deg2rad * hFoshanHipExoskeleton.leftOffsetedAngle) * 0.25 * \
                                              hFoshanHipExoskeleton.leftDirection;
      LowPassFilter_Update(&hFoshanHipExoskeleton.assistiveTorqueFilteredLeft, hFoshanHipExoskeleton.assistiveTorqueLeft);
      LETECH_MG_CurrentControl(&(hFoshanHipExoskeleton.hMotorLeft), hFoshanHipExoskeleton.assistiveTorqueFilteredLeft.output.f / hFoshanHipExoskeleton.hMotorLeft.kt);
      hFoshanHipExoskeleton.switchtask = 1;
    }
    else if (hFoshanHipExoskeleton.switchtask == 1)
    {
      LETECH_MG_ReadAngleSingleTurn(&(hFoshanHipExoskeleton.hMotorLeft));
      hFoshanHipExoskeleton.switchtask = 0;
    }
    
  }
  else if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_RESIST)
  {
    if (hFoshanHipExoskeleton.switchtask == 0)
    {
      hFoshanHipExoskeleton.resistiveTorqueLeft = -hFoshanHipExoskeleton.springFactor * sinf(deg2rad * hFoshanHipExoskeleton.leftOffsetedAngle) * \
                                                hFoshanHipExoskeleton.leftDirection;
      LowPassFilter_Update(&hFoshanHipExoskeleton.resistiveTorqueFilteredLeft, hFoshanHipExoskeleton.resistiveTorqueLeft);
      LETECH_MG_CurrentControl(&(hFoshanHipExoskeleton.hMotorLeft), hFoshanHipExoskeleton.resistiveTorqueFilteredLeft.output.f / hFoshanHipExoskeleton.hMotorLeft.kt);
      hFoshanHipExoskeleton.switchtask = 1;
    }
    else if (hFoshanHipExoskeleton.switchtask == 1)
    {
      LETECH_MG_ReadAngleSingleTurn(&(hFoshanHipExoskeleton.hMotorLeft));
      hFoshanHipExoskeleton.switchtask = 0;
    }
   }
}
