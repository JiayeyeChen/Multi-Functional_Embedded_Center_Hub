#include "foshan_hip_exoskeleton.h"
FoshanHipExoskeletonHandle hFoshanHipExoskeleton;

void FOSHANHIPEXOSKELETON_Init(float loop_duration_second)
{
	hFoshanHipExoskeleton.hMotorLeft = CYBERGEAR_Create(&hcan2, 0x7E, 0, -1.0f);
	hFoshanHipExoskeleton.hMotorRight = CYBERGEAR_Create(&hcan2, 0x7F, 0, 1.0f);
  hFoshanHipExoskeleton.task = FOSHAN_HIP_EXOSKELETON_TASK_NONE;
  hFoshanHipExoskeleton.gravityFactor = 0.0f;
  hFoshanHipExoskeleton.innertiaFactor = 0.0f;
  hFoshanHipExoskeleton.springFactor = 0.0f;
  hFoshanHipExoskeleton.leftAngleOffset = 60.0f;
  hFoshanHipExoskeleton.leftDirection = -1.0f;
  hFoshanHipExoskeleton.switchtask = 0;
	
	hFoshanHipExoskeleton.assistiveTorqueLeft = 0.0f;
	hFoshanHipExoskeleton.assistiveTorqueRight = 0.0f;
	hFoshanHipExoskeleton.resistiveTorqueLeft = 0.0f;
	hFoshanHipExoskeleton.resistiveTorqueRight = 0.0f;
  LowPassFilter_Init(&hFoshanHipExoskeleton.assistiveTorqueFilteredLeft, 2.0f, loop_duration_second);
  LowPassFilter_Init(&hFoshanHipExoskeleton.resistiveTorqueFilteredLeft, 2.0f, loop_duration_second);
	LowPassFilter_Init(&hFoshanHipExoskeleton.assistiveTorqueFilteredRight, 2.0f, loop_duration_second);
  LowPassFilter_Init(&hFoshanHipExoskeleton.resistiveTorqueFilteredRight, 2.0f, loop_duration_second);
}

void FOSHANHIPEXOSKELETON_CentreControl(void)
{
  if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_NONE)
	{
		CYBERGEAR_Disable(&hFoshanHipExoskeleton.hMotorLeft);
		CYBERGEAR_Disable(&hFoshanHipExoskeleton.hMotorRight);
	}
  else if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_ASSIST)
  {
		hFoshanHipExoskeleton.assistiveTorqueLeft = hFoshanHipExoskeleton.gravityFactor * sinf(deg2rad * hFoshanHipExoskeleton.hMotorLeft.realPosDeg.f) * 0.25f * hFoshanHipExoskeleton.hMotorLeft.directionCorrection;
		LowPassFilter_Update(&hFoshanHipExoskeleton.assistiveTorqueFilteredLeft, hFoshanHipExoskeleton.assistiveTorqueLeft);
		CYBERGEAR_GeneralControl(&hFoshanHipExoskeleton.hMotorLeft, hFoshanHipExoskeleton.assistiveTorqueFilteredLeft.output.f, 0.0f, 0.0f, 0.0f, 0.0f);
		
		hFoshanHipExoskeleton.assistiveTorqueRight = hFoshanHipExoskeleton.gravityFactor * sinf(deg2rad * hFoshanHipExoskeleton.hMotorRight.realPosDeg.f) * 0.25f * hFoshanHipExoskeleton.hMotorRight.directionCorrection;
		LowPassFilter_Update(&hFoshanHipExoskeleton.assistiveTorqueFilteredRight, hFoshanHipExoskeleton.assistiveTorqueRight);
		CYBERGEAR_GeneralControl(&hFoshanHipExoskeleton.hMotorRight, hFoshanHipExoskeleton.assistiveTorqueFilteredRight.output.f, 0.0f, 0.0f, 0.0f, 0.0f);
		
//////    if (hFoshanHipExoskeleton.switchtask == 0)
//////    {
//////      hFoshanHipExoskeleton.assistiveTorqueLeft = hFoshanHipExoskeleton.gravityFactor * sinf(deg2rad * hFoshanHipExoskeleton.leftOffsetedAngle) * 0.25f * \
//////                                              hFoshanHipExoskeleton.leftDirection;
//////      LowPassFilter_Update(&hFoshanHipExoskeleton.assistiveTorqueFilteredLeft, hFoshanHipExoskeleton.assistiveTorqueLeft);
//////      LETECH_MG_CurrentControl(&(hFoshanHipExoskeleton.hMotorLeft), hFoshanHipExoskeleton.assistiveTorqueFilteredLeft.output.f / hFoshanHipExoskeleton.hMotorLeft.kt);
//////      hFoshanHipExoskeleton.switchtask = 1;
//////    }
//////    else if (hFoshanHipExoskeleton.switchtask == 1)
//////    {
//////      LETECH_MG_ReadAngleSingleTurn(&(hFoshanHipExoskeleton.hMotorLeft));
//////      hFoshanHipExoskeleton.switchtask = 0;
//////    }
    
  }
  else if (hFoshanHipExoskeleton.task == FOSHAN_HIP_EXOSKELETON_TASK_RESIST)
  {
//    if (hFoshanHipExoskeleton.switchtask == 0)
//    {
//      hFoshanHipExoskeleton.resistiveTorqueLeft = -hFoshanHipExoskeleton.springFactor * sinf(deg2rad * hFoshanHipExoskeleton.leftOffsetedAngle) * \
//                                                hFoshanHipExoskeleton.leftDirection;
//      LowPassFilter_Update(&hFoshanHipExoskeleton.resistiveTorqueFilteredLeft, hFoshanHipExoskeleton.resistiveTorqueLeft);
//      LETECH_MG_CurrentControl(&(hFoshanHipExoskeleton.hMotorLeft), hFoshanHipExoskeleton.resistiveTorqueFilteredLeft.output.f / hFoshanHipExoskeleton.hMotorLeft.kt);
//      hFoshanHipExoskeleton.switchtask = 1;
//    }
//    else if (hFoshanHipExoskeleton.switchtask == 1)
//    {
//      LETECH_MG_ReadAngleSingleTurn(&(hFoshanHipExoskeleton.hMotorLeft));
//      hFoshanHipExoskeleton.switchtask = 0;
//    }
   }
}
