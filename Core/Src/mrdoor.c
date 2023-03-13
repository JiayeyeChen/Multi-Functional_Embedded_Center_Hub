#include "mrdoor.h"

AK10_9HandleCubaMarsFW hAKMotorMrDoorLeft, hAKMotorMrDoorRight;
BENMOKEJI_M15_Handle   hBENMOKEJIMrDoorLeft, hBENMOKEJIMrDoorRight;
float MrDoorRealWeightLeft, MrDoorRealWeightRight;
float MrDoorControllingSupportWeight;

void MRDOOR_MotorInit(void)
{
	
	hAKMotorMrDoorLeft.hcan = &hcan2;
  hAKMotorMrDoorLeft.canID = CAN_ID_TMOTOR_MRDOOR_LEFT_TX;
  hAKMotorMrDoorLeft.lastReceivedTime = 0;
  hAKMotorMrDoorLeft.status = AK10_9_Offline;
  hAKMotorMrDoorLeft.kt = 1.0f;
  hAKMotorMrDoorLeft.accAvgPtr = 0;
  hAKMotorMrDoorLeft.posOffsetDeg = 0.0f;
  hAKMotorMrDoorLeft.posOffsetRad = hAKMotorMrDoorLeft.posOffsetDeg * deg2rad;
  hAKMotorMrDoorLeft.posDirectionCorrection = 1.0f;
  hAKMotorMrDoorLeft.setPos.f = 0.0f;
  hAKMotorMrDoorLeft.setVel.f = 0.0f;
  hAKMotorMrDoorLeft.setIq.f = 0.0f;
  hAKMotorMrDoorLeft.setKp.f = 0.0f;
  hAKMotorMrDoorLeft.setKd.f = 0.0f;
  hAKMotorMrDoorLeft.goalPos.f = 0.0f;
  hAKMotorMrDoorLeft.goalVel.f = 0.0f;
  hAKMotorMrDoorLeft.goalIq.f = 0.0f;
  hAKMotorMrDoorLeft.goalKp.f = 0.0f;
  hAKMotorMrDoorLeft.goalKd.f = 0.0f;
  hAKMotorMrDoorLeft.realAccelerationFiltered.f = 0.0f;
  hAKMotorMrDoorLeft.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorMrDoorLeft.realAccelerationRaw.f = 0.0f;
  hAKMotorMrDoorLeft.cutOffFrequency = 14.043;
  hAKMotorMrDoorLeft.timeDuration = 1.0f / 500.0f;
  hAKMotorMrDoorLeft.alpha = hAKMotorMrDoorLeft.cutOffFrequency * hAKMotorMrDoorLeft.timeDuration / (1.0f + hAKMotorMrDoorLeft.cutOffFrequency * hAKMotorMrDoorLeft.timeDuration);
  hAKMotorMrDoorLeft.ifCustomizedPositionSpeedControlFinished = 1;
  hAKMotorMrDoorLeft.ifMITModeParameterSmootherWorkFinished = 0;
  //cut-off frequency = 4 hz sampling rate 1000hz(no problem)
  hAKMotorMrDoorLeft.a2Butter = -1.9645;
  hAKMotorMrDoorLeft.a3Butter = 0.9651;
  hAKMotorMrDoorLeft.b1Butter = 0.0001551;
  hAKMotorMrDoorLeft.b2Butter = 0.0003103;
  hAKMotorMrDoorLeft.b3Butter = 0.0001551;
  hAKMotorMrDoorLeft.enablingStatus = AK10_9_MITMODE_DISABLED;
	
	
	hAKMotorMrDoorRight.hcan = &hcan2;
  hAKMotorMrDoorRight.canID = CAN_ID_TMOTOR_MRDOOR_RIGHT_TX;
  hAKMotorMrDoorRight.lastReceivedTime = 0;
  hAKMotorMrDoorRight.status = AK10_9_Offline;
  hAKMotorMrDoorRight.kt = 1.0f;
  hAKMotorMrDoorRight.accAvgPtr = 0;
  hAKMotorMrDoorRight.posOffsetDeg = 0.0f;
  hAKMotorMrDoorRight.posOffsetRad = hAKMotorMrDoorRight.posOffsetDeg * deg2rad;
  hAKMotorMrDoorRight.posDirectionCorrection = 1.0f;
  hAKMotorMrDoorRight.setPos.f = 0.0f;
  hAKMotorMrDoorRight.setVel.f = 0.0f;
  hAKMotorMrDoorRight.setIq.f = 0.0f;
  hAKMotorMrDoorRight.setKp.f = 0.0f;
  hAKMotorMrDoorRight.setKd.f = 0.0f;
  hAKMotorMrDoorRight.goalPos.f = 0.0f;
  hAKMotorMrDoorRight.goalVel.f = 0.0f;
  hAKMotorMrDoorRight.goalIq.f = 0.0f;
  hAKMotorMrDoorRight.goalKp.f = 0.0f;
  hAKMotorMrDoorRight.goalKd.f = 0.0f;
  hAKMotorMrDoorRight.realAccelerationFiltered.f = 0.0f;
  hAKMotorMrDoorRight.realAccelerationFilteredPrevious = 0.0f;
  hAKMotorMrDoorRight.realAccelerationRaw.f = 0.0f;
  hAKMotorMrDoorRight.cutOffFrequency = 14.043;
  hAKMotorMrDoorRight.timeDuration = 1.0f / 500.0f;
  hAKMotorMrDoorRight.alpha = hAKMotorMrDoorRight.cutOffFrequency * hAKMotorMrDoorRight.timeDuration / (1.0f + hAKMotorMrDoorRight.cutOffFrequency * hAKMotorMrDoorRight.timeDuration);
  hAKMotorMrDoorRight.ifCustomizedPositionSpeedControlFinished = 1;
  hAKMotorMrDoorRight.ifMITModeParameterSmootherWorkFinished = 0;
  //cut-off frequency = 4 hz sampling rate 1000hz(no problem)
  hAKMotorMrDoorRight.a2Butter = -1.9645;
  hAKMotorMrDoorRight.a3Butter = 0.9651;
  hAKMotorMrDoorRight.b1Butter = 0.0001551;
  hAKMotorMrDoorRight.b2Butter = 0.0003103;
  hAKMotorMrDoorRight.b3Butter = 0.0001551;
  hAKMotorMrDoorRight.enablingStatus = AK10_9_MITMODE_DISABLED;
  
  BENMOKEJI_M15_Init(&hBENMOKEJIMrDoorLeft, &hcan2, 1);
  BENMOKEJI_M15_Init(&hBENMOKEJIMrDoorRight, &hcan2, 2);
}

void MRDOOR_CalculateWeight(AK10_9HandleCubaMarsFW* hmotor_left, \
                            AK10_9HandleCubaMarsFW* hmotor_right, \
                            float k_left, float k_right, float b_left, float b_right)
{
  MrDoorRealWeightLeft = hmotor_left->realCurrent.f * k_left + b_left;
  MrDoorRealWeightRight = hmotor_right->realCurrent.f * k_right + b_right;
}
