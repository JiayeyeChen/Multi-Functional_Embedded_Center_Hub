#include "tmotor_ak10-9_v2_dmfw.h"

const float p_min = -50.2654824574f, p_max = 50.2654824574f, \
        v_min = -50.2654824574f, v_max = 50.2654824574f, \
        i_min = -60.0f, i_max = 60.0f;
  
const float kp_min = 0.0f, kp_max = 500.0f,\
        kd_min = 0.0f, kd_max = 5.0f;

void AK10_9_DMFW_EnableMotor(AK10_9HandleDMFW* hmotor)
{
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
  if (hmotor->controlMode == AK10_9_DM_FW_MODE_MIT)
    hmotor->txHeader.StdId = hmotor->canID;
  else if (hmotor->controlMode == AK10_9_DM_FW_MODE_VELOCITY)
    hmotor->txHeader.StdId = 0x200 + hmotor->canID;
  else if (hmotor->controlMode == AK10_9_DM_FW_MODE_POSITION)
    hmotor->txHeader.StdId = 0x100 + hmotor->canID;
  hmotor->txBuf[0] = 0xFF;
  hmotor->txBuf[1] = 0xFF;
  hmotor->txBuf[2] = 0xFF;
  hmotor->txBuf[3] = 0xFF;
  hmotor->txBuf[4] = 0xFF;
  hmotor->txBuf[5] = 0xFF;
  hmotor->txBuf[6] = 0xFF;
  hmotor->txBuf[7] = 0xFC;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
  
  AK10_9_DMFW_MITMode_ZeroingControlParameters(hmotor);
  hmotor->enablingStatus = AK10_9_MITMODE_ENABLED;
}
void AK10_9_DMFW_DisableMotor(AK10_9HandleDMFW* hmotor)
{
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
  if (hmotor->controlMode == AK10_9_DM_FW_MODE_MIT)
    hmotor->txHeader.StdId = hmotor->canID;
  else if (hmotor->controlMode == AK10_9_DM_FW_MODE_VELOCITY)
    hmotor->txHeader.StdId = 0x200 + hmotor->canID;
  else if (hmotor->controlMode == AK10_9_DM_FW_MODE_POSITION)
    hmotor->txHeader.StdId = 0x100 + hmotor->canID;
  hmotor->txBuf[0] = 0xFF;
  hmotor->txBuf[1] = 0xFF;
  hmotor->txBuf[2] = 0xFF;
  hmotor->txBuf[3] = 0xFF;
  hmotor->txBuf[4] = 0xFF;
  hmotor->txBuf[5] = 0xFF;
  hmotor->txBuf[6] = 0xFF;
  hmotor->txBuf[7] = 0xFD;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
  
  AK10_9_DMFW_MITMode_ZeroingControlParameters(hmotor);
  hmotor->enablingStatus = AK10_9_MITMODE_DISABLED;
}

void AK10_9_DMFW_Zeroing(AK10_9HandleDMFW* hmotor)
{
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.IDE = 0;
  hmotor->txHeader.RTR = 0;
  hmotor->txHeader.StdId = hmotor->canID;
  hmotor->txBuf[0] = 0xFF;
  hmotor->txBuf[1] = 0xFF;
  hmotor->txBuf[2] = 0xFF;
  hmotor->txBuf[3] = 0xFF;
  hmotor->txBuf[4] = 0xFF;
  hmotor->txBuf[5] = 0xFF;
  hmotor->txBuf[6] = 0xFF;
  hmotor->txBuf[7] = 0xFE;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_DMFW_MITMode_ZeroingControlParameters(AK10_9HandleDMFW* hmotor)
{
  hmotor->setPos.f = 0.0f;
  hmotor->goalPos.f = 0.0f;
  hmotor->setVel.f = 0.0f;
  hmotor->goalVel.f = 0.0f;
  hmotor->setIq.f = 0.0f;
  hmotor->goalIq.f = 0.0f;
  hmotor->setKp.f = 0.0f;
  hmotor->goalKp.f = 0.0f;
  hmotor->setKd.f = 0.0f;
  hmotor->goalKd.f = 0.0f;
  hmotor->ifMITModeParameterSmootherWorkFinished = 1;
}

void AK10_9_DMFW_MotorStatusMonitor(AK10_9HandleDMFW* hmotor, uint32_t timeout_ms)
{
  if ((HAL_GetTick() - hmotor->lastReceivedTime) > timeout_ms)
    hmotor->status = AK10_9_Offline;
  else
    hmotor->status = AK10_9_Online;
}

void AK10_9_DMFW_MITModeControl_Rad(AK10_9HandleDMFW* hmotor, float pos, float vel, float kp, float kd, float iq)
{
  hmotor->setPos.f = pos;
  hmotor->setVel.f = vel;
  hmotor->setIq.f = iq;
  hmotor->setKp.f = kp;
  hmotor->setKd.f = kd;
  
  pos = MIN(MAX(pos, p_min), p_max);
  pos *= hmotor->posDirectionCorrection;
  vel = MIN(MAX(vel, v_min), v_max);
  vel *= hmotor->posDirectionCorrection;
  kp = MIN(MAX(kp, kp_min), kp_max);
  kd = MIN(MAX(kd, kd_min), kd_max);
  iq = MIN(MAX(iq, i_min), i_max);
  iq *= hmotor->posDirectionCorrection;
  
  uint16_t pInt = AK10_9_DMFW_FloatToUint(pos, p_min, p_max, 16);
  uint16_t vInt = AK10_9_DMFW_FloatToUint(vel, v_min, v_max, 12);
  uint16_t kpInt = AK10_9_DMFW_FloatToUint(kp, kp_min, kp_max, 12);
  uint16_t kdInt = AK10_9_DMFW_FloatToUint(kd, kd_min, kd_max, 12);
  uint16_t iInt = AK10_9_DMFW_FloatToUint(iq, i_min, i_max, 12);
  
  hmotor->txBuf[0] = pInt >> 8;
  hmotor->txBuf[1] = pInt & 0xFF;
  hmotor->txBuf[2] = vInt >> 4;
  hmotor->txBuf[3] = ((vInt & 0x0F) << 4) | (kpInt >> 8);
  hmotor->txBuf[4] = kpInt & 0xFF;
  hmotor->txBuf[5] = kdInt >> 4;
  hmotor->txBuf[6] = ((kdInt & 0x0F) << 4) | (iInt >> 8);
  hmotor->txBuf[7] = iInt & 0xFF;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_DMFW_MITModeControl_Deg(AK10_9HandleDMFW* hmotor, float pos, float vel, float kp, float kd, float iq)
{
  float pos_rad, vel_rad;
  pos_rad = pos * deg2rad;
  vel_rad = vel * deg2rad;
  AK10_9_DMFW_MITModeControl_Rad(hmotor, pos_rad, vel_rad, kp, kd, iq);
}

void AK10_9_DMFW_MITModeCurrentControl(AK10_9HandleDMFW* hmotor, float iq)
{
  AK10_9_DMFW_MITModeControl_Rad(hmotor, 0.0f, 0.0f, 0.0f, 0.0f, iq);
}

void AK10_9_DMFW_MITMode_ContinuousControlManager(AK10_9HandleDMFW* hmotor, \
                                                  float pos_slope_deg, float vel_slope_deg, float iq_slope, \
                                                  float kp_slope, float kd_slope, float loop_duration_ms)
{
  if (hmotor->enablingStatus == AK10_9_MITMODE_ENABLED)
  {
    float diff_pos = hmotor->goalPos.f - hmotor->setPos.f, abs_diff_pos = fabs(diff_pos);
    float diff_vel = hmotor->goalVel.f - hmotor->setVel.f, abs_diff_vel = fabs(diff_vel);
    float diff_kp =  hmotor->goalKp.f - hmotor->setKp.f, abs_diff_kp = fabs(diff_kp);
    float diff_kd =  hmotor->goalKd.f - hmotor->setKd.f, abs_diff_kd = fabs(diff_kd);
    float diff_iq =  hmotor->goalIq.f - hmotor->setIq.f, abs_diff_iq = fabs(diff_iq);
    /* Position smoother */
    if (fabs(diff_pos) > 5.0f * deg2rad)//5 Deg
      hmotor->setPos.f += (diff_pos / abs_diff_pos) * pos_slope_deg * deg2rad * loop_duration_ms;
    else
      hmotor->setPos.f = hmotor->goalPos.f;
    /* Velocity smoother */
    if (fabs(diff_vel) > 5.0f * deg2rad)//5 Deg/sec
      hmotor->setVel.f += (diff_vel / abs_diff_vel) * vel_slope_deg * deg2rad * loop_duration_ms;
    else
      hmotor->setVel.f = hmotor->goalVel.f;
    /* Iq smoother */
    if (fabs(diff_iq) > 1.0f)
      hmotor->setIq.f += (diff_iq / abs_diff_iq) * iq_slope * loop_duration_ms;
    else
      hmotor->setIq.f = hmotor->goalIq.f;
    /* Kp smoother */
    if (fabs(diff_kp) > 1.0f)
      hmotor->setKp.f += (diff_kp / abs_diff_kp) * kp_slope * loop_duration_ms;
    else
      hmotor->setKp.f = hmotor->goalKp.f;
    /* Kd smoother */
    if (fabs(diff_kd) > 1.0f)
      hmotor->setKd.f += (diff_kd / abs_diff_kd) * kd_slope * loop_duration_ms;
    else
      hmotor->setKd.f = hmotor->goalKd.f;
    
    AK10_9_DMFW_MITModeControl_Rad(hmotor, hmotor->setPos.f, hmotor->setVel.f, hmotor->setKp.f, hmotor->setKd.f, hmotor->setIq.f);
  }
  else if (hmotor->enablingStatus == AK10_9_MITMODE_DISABLED)
  {
  }
}

void AK10_9_DMFW_MITMode_ContinuousControl_Rad(AK10_9HandleDMFW* hmotor, float goal_pos, float goal_vel, \
                                                 float goal_kp, float goal_kd, float goal_iq)
{
  hmotor->goalPos.f = goal_pos;
  hmotor->goalVel.f = goal_vel;
  hmotor->goalKp.f = goal_kp;
  hmotor->goalKd.f = goal_kd;
  hmotor->goalIq.f = goal_iq;
  
  float diff_pos = hmotor->goalPos.f - hmotor->setPos.f, abs_diff_pos = fabs(diff_pos);
  float diff_vel = hmotor->goalVel.f - hmotor->setVel.f, abs_diff_vel = fabs(diff_vel);
  float diff_kp =  hmotor->goalKp.f - hmotor->setKp.f, abs_diff_kp = fabs(diff_kp);
  float diff_kd =  hmotor->goalKd.f - hmotor->setKd.f, abs_diff_kd = fabs(diff_kd);
  float diff_iq =  hmotor->goalIq.f - hmotor->setIq.f, abs_diff_iq = fabs(diff_iq);
  
  if (abs_diff_pos <= deg2rad && abs_diff_vel <= deg2rad && abs_diff_kp <= 1.0f && abs_diff_kd <= 1.0f && abs_diff_iq <= 1.0f)
    hmotor->ifMITModeParameterSmootherWorkFinished = 1;
  else
    hmotor->ifMITModeParameterSmootherWorkFinished = 0;
}

void AK10_9_DMFW_MITMode_ContinuousControl_Deg(AK10_9HandleDMFW* hmotor, float goal_pos, float goal_vel, \
                                                 float goal_kp, float goal_kd, float goal_iq)
{
  float goal_pos_rad, goal_vel_rad;
  goal_pos_rad = goal_pos * deg2rad;
  goal_vel_rad = goal_vel * deg2rad;
  
  AK10_9_DMFW_MITMode_ContinuousControl_Rad(hmotor, goal_pos_rad, goal_vel_rad, goal_kp, goal_kd, goal_iq);
}

void AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(AK10_9HandleDMFW* hmotor, float goal_pos, float goal_vel, \
                                                         float goal_kp, float goal_kd, float goal_iq)
{
  AK10_9_DMFW_MITMode_ContinuousControl_Deg(hmotor, goal_pos + hmotor->posOffsetDeg, goal_vel, goal_kp, goal_kd, goal_iq);
}

float AK10_9_DMFW_MITMode_CustomizedVelocityPIDControlCalSetIq_Deg(AK10_9HandleDMFW* hmotor, PIDHandle* hpid, float desVal)
{
  float setIq;
  PID(&setIq, hmotor->realVelocityPresent.f, desVal, hpid);
  return setIq;
}

uint16_t AK10_9_DMFW_FloatToUint(float x, float x_min, float x_max, uint16_t bits)
{
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (uint16_t)((x - x_min) * ((float)((1<<bits)/span)));
}

float AK10_9_DMFW_UintToFloat(uint16_t x_int, float x_min, float x_max, uint16_t bits)
{
  float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

void AK10_9_DMFW_PositionVelocityControl(AK10_9HandleDMFW* hmotor, float pos, float vel)
{
  hmotor->setPos.f = pos;
  hmotor->setVel.f = vel;
  hmotor->txHeader.IDE = CAN_ID_STD;
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.StdId = 0x100 + hmotor->canID;
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union FloatUInt8 temVEL, temPOS;
  temPOS.f = pos;
  hmotor->txBuf[0] = temPOS.b8[0];
  hmotor->txBuf[1] = temPOS.b8[1];
  hmotor->txBuf[2] = temPOS.b8[2];
  hmotor->txBuf[3] = temPOS.b8[3];
  temVEL.f = vel;
  hmotor->txBuf[4] = temVEL.b8[0];
  hmotor->txBuf[5] = temVEL.b8[1];
  hmotor->txBuf[6] = temVEL.b8[2];
  hmotor->txBuf[7] = temVEL.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}
void AK10_9_DMFW_VelocityControl(AK10_9HandleDMFW* hmotor, float vel)
{
  hmotor->setVel.f = vel;
  hmotor->txHeader.IDE = CAN_ID_STD;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.StdId = 0x200 + hmotor->canID;
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union FloatUInt8 temVEL;
  temVEL.f = vel;
  hmotor->txBuf[0] = temVEL.b8[0];
  hmotor->txBuf[1] = temVEL.b8[1];
  hmotor->txBuf[2] = temVEL.b8[2];
  hmotor->txBuf[3] = temVEL.b8[3];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_DMFW_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleDMFW* hmotor, uint8_t rxbuf[])
{
  uint16_t pUint, vUint, iUint;
  pUint=(rxbuf[1] << 8) | rxbuf[2];
  vUint=(rxbuf[3] << 4) | (rxbuf[4] >> 4);
  iUint=((rxbuf[4] & 0xF) << 8) | rxbuf[5];
  
  hmotor->realPositionRad.f = AK10_9_DMFW_UintToFloat(pUint, p_min,p_max, 16);
  hmotor->realPositionRad.f *= hmotor->posDirectionCorrection;
  hmotor->realPositionDeg.f = hmotor->realPositionRad.f * 180.0f / pi;
  hmotor->realPositionOffseted.f = hmotor->realPositionDeg.f - hmotor->posOffsetDeg;
  hmotor->realPositionOffsetedRad.f = hmotor->realPositionOffseted.f * deg2rad;
  hmotor->realVelocityPresentRad.f = AK10_9_DMFW_UintToFloat(vUint, v_min, v_max, 12);
  hmotor->realVelocityPresentRad.f *= hmotor->posDirectionCorrection;
  hmotor->realVelocityPresent.f = hmotor->realVelocityPresentRad.f * rad2deg;
  hmotor->realAccelerationRaw.f = (hmotor->realVelocityPresent.f - hmotor->realVelocityPrevious[0].f) / 0.001f;
  hmotor->realVelocityPrevious[0].f = hmotor->realVelocityPresent.f;
  hmotor->realCurrent.f  = AK10_9_DMFW_UintToFloat(iUint, i_min, i_max, 12);
  hmotor->realCurrent.f *= hmotor->posDirectionCorrection;
  hmotor->realTorque.f = hmotor->realCurrent.f * hmotor->kt;
  
  //Butterworth filter method to estimate acceleration//
  hmotor->realAccelerationFiltered.f = hmotor->b1Butter * hmotor->realAccelerationRaw.f + \
                                       hmotor->b2Butter * hmotor->realAccelerationRawPreviousButter[0] + \
                                       hmotor->b3Butter * hmotor->realAccelerationRawPreviousButter[1] - \
                                       hmotor->a2Butter * hmotor->realAccelerationFilteredPreviousButter[0] - \
                                       hmotor->a3Butter * hmotor->realAccelerationFilteredPreviousButter[1];
  hmotor->realAccelerationRawPreviousButter[1] = hmotor->realAccelerationRawPreviousButter[0];
  hmotor->realAccelerationRawPreviousButter[0] = hmotor->realAccelerationRaw.f;
  hmotor->realAccelerationFilteredPreviousButter[1] = hmotor->realAccelerationFilteredPreviousButter[0];
  hmotor->realAccelerationFilteredPreviousButter[0] = hmotor->realAccelerationFiltered.f;
  
  hmotor->realAccelerationFilteredRad.f = hmotor->realAccelerationFiltered.f * deg2rad;
  //////////////////////////////////////////////////////
  hmotor->lastReceivedTime = HAL_GetTick();
}
