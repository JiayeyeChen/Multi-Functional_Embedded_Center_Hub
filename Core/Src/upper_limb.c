#include "upper_limb.h"

UpperLimbHandle hUpperLimb;

float pid_speed_control_set_val = 0.0f;

void UPPERLIMB_Init(void)
{
  hUpperLimb.hcan = &hcan2;
  hUpperLimb.mainTask = UPPERLIMB_MAIN_TASK_NONE;
}

void UPPERLIMB_CANGetForceFeedback(UpperLimbHandle* hupperlimb, uint8_t data[])
{
  uint8_t ptr = 0;
  hupperlimb->forceXRaw.b8[0] = data[ptr++];
  hupperlimb->forceXRaw.b8[1] = data[ptr++];
  hupperlimb->forceYRaw.b8[0] = data[ptr++];
  hupperlimb->forceYRaw.b8[1] = data[ptr++];
  hupperlimb->forceZRaw.b8[0] = data[ptr++];
  hupperlimb->forceZRaw.b8[1] = data[ptr++];
  
  hupperlimb->forceX.f = (((float)hupperlimb->forceXRaw.b16 - 0.5f) * 152.0f) * 0.000001f;
  hupperlimb->forceY.f = (((float)hupperlimb->forceYRaw.b16 - 0.5f) * 152.0f) * 0.000001f;
  hupperlimb->forceZ.f = (((float)hupperlimb->forceZRaw.b16 - 0.5f) * 152.0f) * 0.000001f;
}



void UPPERLIMB_CANRequestForceData(UpperLimbHandle* hupperlimb)
{
  hupperlimb->txHeader.DLC = 1;
  hupperlimb->txHeader.IDE = 0;
  hupperlimb->txHeader.RTR = 0;
  hupperlimb->txHeader.StdId = CAN_ID_UPPER_LIMB_FORCE_SENSOR_TX;
  hupperlimb->canTxBuf[0] = 0xFF;
  HAL_CAN_AddTxMessage(hupperlimb->hcan, &hupperlimb->txHeader, hupperlimb->canTxBuf, &hupperlimb->txMailBox);
}

void UPPERLIMB_ControlCenter(void)
{
  hUpperLimb.q21Rad.f = hAKMotorDMFW1.realPositionRad.f;
  hUpperLimb.q21Deg.f = hAKMotorDMFW1.realPositionDeg.f;
  hUpperLimb.q21DotRad.f = hAKMotorDMFW1.realVelocityPresentRad.f;
  hUpperLimb.q21DotDeg.f = hAKMotorDMFW1.realVelocityPresent.f;
  hUpperLimb.q31Rad.f = hAKMotorDMFW2.realPositionRad.f + pi/2.0f;
  hUpperLimb.q31Deg.f = hAKMotorDMFW2.realPositionDeg.f + 90.0f;
  hUpperLimb.q31DotRad.f = hAKMotorDMFW2.realVelocityPresentRad.f;
  hUpperLimb.q31DotDeg.f = hAKMotorDMFW2.realVelocityPresent.f;
  hUpperLimb.q1Rad.f = hAKMotorDMFW3.realPositionRad.f;
  hUpperLimb.q1Deg.f = hAKMotorDMFW3.realPositionDeg.f;
  hUpperLimb.q1DotRad.f = hAKMotorDMFW3.realVelocityPresentRad.f;
  hUpperLimb.q1DotDeg.f = hAKMotorDMFW3.realVelocityPresent.f;
  hUpperLimb.q4Deg.f = 8640.0f - hEncoderUpperLimb1.angleDeg.f;
  if (hUpperLimb.q4Deg.f > 360.0f)
    hUpperLimb.q4Deg.f = 0.0f;
  hUpperLimb.q4Rad.f = hUpperLimb.q4Deg.f * deg2rad;
  
  UPPERLIMB_Calculate_Jacobian(&hUpperLimb);
  
  switch (hUpperLimb.mainTask)
  {
  	case UPPERLIMB_MAIN_TASK_NONE:
      AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW1, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW2, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW3, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  		break;
  	case UPPERLIMB_MAIN_TASL_ADMITTANCE_CONTROL:
  		break;
    case UPPERLIMB_MAIN_TASK_MANUAL_CONTROL:
      UPPERLIMB_EndEffectorSpeedControl(&hUpperLimb);
      break;
    case UPPERLIMB_MAIN_TASK_PID_VELOCITY_CONTROL:
      AK10_9_DMFW_MITMode_ContinuousControl_Deg(hMotorPtrManualControlDMFW, 0.0f, 0.0f, 0.0f, 0.0f, \
      AK10_9_DMFW_MITMode_CustomizedVelocityPIDControlCalSetIq_Deg(hMotorPtrManualControlDMFW, hPIDPtrSpeedControl, pid_speed_control_set_val));
      break;
  	default:
  		break;
  }
  AK10_9_DMFW_MITModeControl_Rad(&hAKMotorDMFW1, 0.0f, 0.0f, 0.0f, 0.0f, hAKMotorDMFW1.setIq.f);
  AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorDMFW1, \
                                                     180.0f, 180.0f, 10.0f, 200.0f, 2.5f, 0.001f);
  AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorDMFW2, \
                                                     180.0f, 180.0f, 10.0f, 200.0f, 2.5f, 0.001f);
  MicroSecDelay(&htim8, 400);
  AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorDMFW3, \
                                                     180.0f, 180.0f, 10.0f, 200.0f, 2.5f, 0.001f);
}

void UPPERLIMB_ForwardKinematics(UpperLimbHandle* hupperlimb)
{
  
}

void UPPERLIMB_InverseKinematics(UpperLimbHandle* hupperlimb)
{
  
}

void UPPERLIMB_Calculate_Jacobian(UpperLimbHandle* hupperlimb)
{
  hupperlimb->delta1 = UPPERLIMB_L4 * cosf(hupperlimb->q31Rad.f) - UPPERLIMB_L21 * cosf(hupperlimb->q21Rad.f);
  hupperlimb->J[0][0] = sinf(hupperlimb->q1Rad.f) * hupperlimb->delta1 + UPPERLIMB_Lf * \
                        cosf(hupperlimb->q4Rad.f) * cosf(hupperlimb->q31Rad.f) * sinf(hupperlimb->q1Rad.f) + \
                        UPPERLIMB_Lf * sinf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q4Rad.f) * sinf(hupperlimb->q31Rad.f);
  hupperlimb->J[0][1] = -UPPERLIMB_L21 * cosf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q21Rad.f);
  hupperlimb->J[0][2] = UPPERLIMB_L4 * cosf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q31Rad.f) \
                        + UPPERLIMB_Lf * cosf(hupperlimb->q1Rad.f) * cosf(hupperlimb->q4Rad.f) * sinf(hupperlimb->q31Rad.f) \
                        - UPPERLIMB_Lf * cos(hupperlimb->q1Rad.f) * cosf(hupperlimb->q31Rad.f) * sinf(hupperlimb->q4Rad.f);
  hupperlimb->J[1][0] = -cosf(hupperlimb->q1Rad.f) * hupperlimb->delta1 - \
                        UPPERLIMB_Lf * cosf(hupperlimb->q1Rad.f) * cosf(hupperlimb->q4Rad.f) * cosf(hupperlimb->q31Rad.f) - \
                        UPPERLIMB_Lf * cosf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q4Rad.f) * sinf(hupperlimb->q31Rad.f);
  hupperlimb->J[1][1] = -UPPERLIMB_L21 * sinf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q21Rad.f);
  hupperlimb->J[1][2] = UPPERLIMB_L4 * sinf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q31Rad.f) + \
                        UPPERLIMB_Lf * cosf(hupperlimb->q4Rad.f) * sinf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q31Rad.f) \
                        - UPPERLIMB_Lf * cosf(hupperlimb->q31Rad.f) * sinf(hupperlimb->q1Rad.f) * sinf(hupperlimb->q4Rad.f);
  hupperlimb->J[2][0] = 0.0f;
  hupperlimb->J[2][1] = UPPERLIMB_L21 * cosf(hupperlimb->q21Rad.f);
  hupperlimb->J[2][2] = UPPERLIMB_Lf * cosf(hupperlimb->q4Rad.f) * cosf(hupperlimb->q31Rad.f) - \
                        UPPERLIMB_L4 * cosf(hupperlimb->q31Rad.f) + \
                        UPPERLIMB_Lf * sinf(hupperlimb->q4Rad.f) * sinf(hupperlimb->q31Rad.f);
  hupperlimb->detJ = DetMatrix3D(hupperlimb->J);
  InverseMatrix3D(hupperlimb->J, hupperlimb->invJ);
}

void UPPERLIMB_EndEffectorSpeedControl(UpperLimbHandle* hupperlimb)
{
  hupperlimb->q1DotRadDes.f = hupperlimb->invJ[0][0] * hupperlimb->endPosDesX.f + \
                              hupperlimb->invJ[0][1] * hupperlimb->endPosDesY.f + \
                              hupperlimb->invJ[0][2] * hupperlimb->endPosDesZ.f;
  
  hupperlimb->q21DotRadDes.f = hupperlimb->invJ[1][0] * hupperlimb->endPosDesX.f + \
                               hupperlimb->invJ[1][1] * hupperlimb->endPosDesY.f + \
                               hupperlimb->invJ[1][2] * hupperlimb->endPosDesZ.f;
  
  hupperlimb->q31DotRadDes.f = hupperlimb->invJ[2][0] * hupperlimb->endPosDesX.f + \
                               hupperlimb->invJ[2][1] * hupperlimb->endPosDesY.f + \
                               hupperlimb->invJ[2][2] * hupperlimb->endPosDesZ.f;
  
  hupperlimb->q1DotDegDes.f = hupperlimb->q1DotRadDes.f * rad2deg;
  hupperlimb->q21DotDegDes.f = hupperlimb->q21DotRadDes.f * rad2deg;
  hupperlimb->q31DotDegDes.f = hupperlimb->q31DotRadDes.f * rad2deg;
  
  AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW3, 0.0f, 0.0f, 0.0f, 0.0f, \
        AK10_9_DMFW_MITMode_CustomizedVelocityPIDControlCalSetIq_Deg(&hAKMotorDMFW3, &hPIDDMMotor3, hupperlimb->q1DotDegDes.f));
  
  AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW2, 0.0f, 0.0f, 0.0f, 0.0f, \
        AK10_9_DMFW_MITMode_CustomizedVelocityPIDControlCalSetIq_Deg(&hAKMotorDMFW2, &hPIDDMMotor2, hupperlimb->q31DotDegDes.f));
        
  AK10_9_DMFW_MITMode_ContinuousControl_Deg(&hAKMotorDMFW1, 0.0f, 0.0f, 0.0f, 0.0f, \
        AK10_9_DMFW_MITMode_CustomizedVelocityPIDControlCalSetIq_Deg(&hAKMotorDMFW1, &hPIDDMMotor1, hupperlimb->q21DotDegDes.f));
}
