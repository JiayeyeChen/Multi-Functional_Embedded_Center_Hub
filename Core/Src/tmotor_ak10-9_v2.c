#include "tmotor_ak10-9_v2.h"

const float p_min = -50.2654824574f, p_max = 50.2654824574f, \
        v_min = -50.2654824574f, v_max = 50.2654824574f, \
        i_min = -60.0f, i_max = 60.0f;
  
const float kp_min = 0.0f, kp_max = 500.0f,\
        kd_min = 0.0f, kd_max = 5.0f;

enum ServoMotorMode_CAN_PACKET_ID
{
  SERVO_CAN_PACKET_SET_DUTY = 0,
  SERVO_CAN_PACKET_SET_CURRENT,
  SERVO_CAN_PACKET_SET_CURRENT_BRAKE,
  SERVO_CAN_PACKET_SET_RPM,
  SERVO_CAN_PACKET_SET_POS,
  SERVO_CAN_PACKET_SET_ORIGIN_HERE,
  SERVO_CAN_PACKET_SET_POS_SPD
};

//float current: -60A~60A
void AK10_9_ServoMode_CurrentControl(AK10_9HandleCubaMarsFW* hmotor, float current)
{
  hmotor->setIq.f = current;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_CURRENT << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  union Int32UInt8 temCurrent;
  temCurrent.b32 = (int32_t)(hmotor->setIq.f * 1000.0f);
  hmotor->txBuf[0] = temCurrent.b8[3];
  hmotor->txBuf[1] = temCurrent.b8[2];
  hmotor->txBuf[2] = temCurrent.b8[1];
  hmotor->txBuf[3] = temCurrent.b8[0];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

//float speed: -10000epm~10000epm
void AK10_9_ServoMode_VelocityControl(AK10_9HandleCubaMarsFW* hmotor, float speed)
{
  hmotor->setVel.f = speed;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_RPM << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temSPD;
  temSPD.b32 = (int32_t)hmotor->setVel.f * 60.0f * 21.0f * 9.0f / 360.0f;;
  hmotor->txBuf[0] = temSPD.b8[3];
  hmotor->txBuf[1] = temSPD.b8[2];
  hmotor->txBuf[2] = temSPD.b8[1];
  hmotor->txBuf[3] = temSPD.b8[0];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

//float position: -3600deg~3600deg
void AK10_9_ServoMode_PositionControl(AK10_9HandleCubaMarsFW* hmotor, float position)
{
  hmotor->setPos.f = position;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 4;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_POS << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temPOS;
  temPOS.b32 = (int32_t)(hmotor->setPos.f * 10000.0f);
  hmotor->txBuf[0] = temPOS.b8[3];
  hmotor->txBuf[1] = temPOS.b8[2];
  hmotor->txBuf[2] = temPOS.b8[1];
  hmotor->txBuf[3] = temPOS.b8[0];
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_PositionSpeenControlCustomized(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float loop_duration)
{
  if (hmotor->ifCustomizedPositionSpeedControlFinished)
  {
    hmotor->setPos.f = hmotor->realPosition.f;
    hmotor->ifCustomizedPositionSpeedControlFinished = 0;
  }
  float setPosIncrement = speed * loop_duration;
  if (fabs((double)(hmotor->setPos.f - position)) >= 1.0f)
  {
    if (position > hmotor->setPos.f)
      AK10_9_ServoMode_PositionControl(hmotor, hmotor->setPos.f + setPosIncrement);
    else
      AK10_9_ServoMode_PositionControl(hmotor, hmotor->setPos.f - setPosIncrement);
  }
  else
  {
    AK10_9_ServoMode_PositionControl(hmotor, position);
    hmotor->ifCustomizedPositionSpeedControlFinished = 1;
  }
}

void AK10_9_ServoMode_PositionControlWithOffset(AK10_9HandleCubaMarsFW* hmotor, float position)
{
  AK10_9_ServoMode_PositionControl(hmotor, position * hmotor->posDirectionCorrection + hmotor->posOffsetDeg);
}
void AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float loop_duration)
{
  AK10_9_ServoMode_PositionSpeenControlCustomized(hmotor, position * hmotor->posDirectionCorrection + hmotor->posOffsetDeg, \
                                                  speed, loop_duration);
}

void AK10_9_ServoMode_PositionSpeedControl(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, int16_t acceleration)
{
  hmotor->setPos.f = position;
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 8;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_POS_SPD << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  union Int32UInt8 temPOS;
  temPOS.b32 = (int32_t)hmotor->setPos.f * 10000.0f;
  hmotor->txBuf[0] = temPOS.b8[3];
  hmotor->txBuf[1] = temPOS.b8[2];
  hmotor->txBuf[2] = temPOS.b8[1];
  hmotor->txBuf[3] = temPOS.b8[0];
  
  union Int16UInt8 temSPD;
  temSPD.b16 = (int16_t)(speed * 60.0f * 21.0f * 9.0f / 3600.0f);
  hmotor->txBuf[4] = temSPD.b8[1];
  hmotor->txBuf[5] = temSPD.b8[0];
  hmotor->txBuf[6] = (uint8_t)((acceleration >> 8) & 0xFF);
  hmotor->txBuf[7] = (uint8_t)(acceleration * 0xFF);
  
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_ServoMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubaMarsFW* hmotor, uint8_t rxbuf[])
{
  hmotor->lastReceivedTime = HAL_GetTick();
  if (rxheader->ExtId == hmotor->canID)
    memcpy(hmotor->rxBuf, rxbuf, 8);
  hmotor->realPosition.f = (float)((int16_t)(hmotor->rxBuf[0] << 8 | hmotor->rxBuf[1]));
  hmotor->realPosition.f /= 10.0f;
  hmotor->realPositionOffseted.f = hmotor->realPosition.f - hmotor->posOffsetDeg;
  hmotor->realPositionOffseted.f *= hmotor->posDirectionCorrection;
  hmotor->realPositionOffsetedRad.f = hmotor->realPositionOffseted.f * deg2rad;
  hmotor->realVelocityPresent.f = (float)((int16_t)(hmotor->rxBuf[2] << 8 | hmotor->rxBuf[3]));
  hmotor->realVelocityPresent.f = (hmotor->realVelocityPresent.f * 3600.0f / 60.0f)  / (21.0f * 9.0f);
  hmotor->realVelocityPresentRad.f = hmotor->realVelocityPresent.f * deg2rad;
  hmotor->realAccelerationRaw.f = (hmotor->realVelocityPresent.f - hmotor->realVelocityPrevious[0].f) / 0.004f;
  hmotor->realVelocityPrevious[0].f = hmotor->realVelocityPrevious[1].f;
  hmotor->realVelocityPrevious[1].f = hmotor->realVelocityPresent.f;
  hmotor->realCurrent.f = (float)((int16_t)(hmotor->rxBuf[4] << 8 | hmotor->rxBuf[5]));
  hmotor->realCurrent.f /= 100.0f;
  hmotor->realTorque.f = hmotor->realCurrent.f * hmotor->kt;
  hmotor->temperature = (int8_t)hmotor->rxBuf[6];
  hmotor->errorCode = hmotor->rxBuf[7];
  
  //Moving average method to estimate acceleration//
//  hmotor->accAverageBuf[hmotor->accAvgPtr++] = hmotor->realAccelerationRaw.f;
//  if (hmotor->accAvgPtr >= SIZE_OF_MOVING_ACC_AVG_BUFFER)
//    hmotor->accAvgPtr = 0;
//  hmotor->accAverage = 0.0f;
//  for (uint8_t i = 0; i < SIZE_OF_MOVING_ACC_AVG_BUFFER; i++)
//    hmotor->accAverage += hmotor->accAverageBuf[i];
//  hmotor->accAverage /= (float)SIZE_OF_MOVING_ACC_AVG_BUFFER;
//  hmotor->realAccelerationFiltered.f = hmotor->accAverage;
  ///////////////////////////////////////////////////
  //Low pass filter method to estimate acceleration//
//  hmotor->realAccelerationFiltered.f = (1.0f - hmotor->alpha) * hmotor->realAccelerationFilteredPrevious + hmotor->alpha * hmotor->realAccelerationRaw.f;
//  hmotor->realAccelerationFilteredPrevious = hmotor->realAccelerationFiltered.f;
  ///////////////////////////////////////////////////
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
}

void AK10_9_ServoMode_Zeroing(AK10_9HandleCubaMarsFW* hmotor)
{
  hmotor->txHeader.IDE = CAN_ID_EXT;
  hmotor->txHeader.DLC = 1;
  hmotor->txHeader.ExtId = (uint32_t)(SERVO_CAN_PACKET_SET_ORIGIN_HERE << 8) | (hmotor->canID & 0xFF);
  hmotor->txHeader.RTR = CAN_RTR_DATA;
  
  hmotor->txBuf[0] = 1;
  HAL_CAN_AddTxMessage(hmotor->hcan, &hmotor->txHeader, hmotor->txBuf, hmotor->pTxMailbox);
}

void AK10_9_MITMode_EnableMotor(AK10_9HandleCubaMarsFW* hmotor)
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
  hmotor->txBuf[7] = 0xFC;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
  
  AK10_9_CubeMarsFW_MITMode_ZeroingControlParameters(hmotor);
  hmotor->enablingStatus = AK10_9_MITMODE_ENABLED;
}
void AK10_9_MITMode_DisableMotor(AK10_9HandleCubaMarsFW* hmotor)
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
  hmotor->txBuf[7] = 0xFD;
  HAL_CAN_AddTxMessage(hmotor->hcan, &(hmotor->txHeader), hmotor->txBuf, hmotor->pTxMailbox);
  
  AK10_9_CubeMarsFW_MITMode_ZeroingControlParameters(hmotor);
  hmotor->enablingStatus = AK10_9_MITMODE_DISABLED;
}

void AK10_9_MITMode_Zeroing(AK10_9HandleCubaMarsFW* hmotor)
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

void AK10_9_CubeMarsFW_MITMode_ZeroingControlParameters(AK10_9HandleCubaMarsFW* hmotor)
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

void AK10_9_MITModeControl_Deg(AK10_9HandleCubaMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq)
{
  float KP_MIN = 0.0f;
  float KP_MAX = 500.0f;
  float KD_MIN = 0.0f;
  float KD_MAX = 5.0f;
  hmotor->setPos.f = pos;
  hmotor->setVel.f = vel;
  hmotor->setIq.f = iq;
  hmotor->setKp.f = kp;
  hmotor->setKd.f = kd;
  kp = MIN(MAX(kp, KP_MIN), KP_MAX);
  kd = MIN(MAX(kd, KD_MIN), KD_MAX);
  
  
  uint16_t pInt = (uint16_t)((hmotor->setPos.f * hmotor->posDirectionCorrection * 32767.0f / 720.0f) + 32767.0f);
  uint16_t vInt = (uint16_t)(hmotor->setVel.f * hmotor->posDirectionCorrection / 1.40625f + 2048.0f);
  uint16_t iInt = (uint16_t)(hmotor->setIq.f * hmotor->posDirectionCorrection / 0.0293111871f + 2048.0f);
  uint16_t kpInt = FloatToUint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kdInt = FloatToUint(kd, KD_MIN, KD_MAX, 12);
  
  
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

void AK10_9_MITModeControl_Rad(AK10_9HandleCubaMarsFW* hmotor, float pos, float vel, float kp, float kd, float iq)
{
  float pos_deg, vel_deg;
  pos_deg = pos * rad2deg;
  vel_deg = vel * rad2deg;
  AK10_9_MITModeControl_Deg(hmotor, pos_deg, vel_deg, kp, kd, iq);
}

void AK10_9_MITModeCurrentControl(AK10_9HandleCubaMarsFW* hmotor, float iq)
{
  AK10_9_MITModeControl_Deg(hmotor, 0.0f, 0.0f, 0.0f, 0.0f, iq);
}

void AK10_9_MITMode_GetFeedbackMsg(CAN_RxHeaderTypeDef* rxheader, AK10_9HandleCubaMarsFW* hmotor, uint8_t rxbuf[])
{
  uint16_t pUint, vUint, iUint;
  pUint=(rxbuf[1] << 8) | rxbuf[2];
  vUint=(rxbuf[3] << 4) | (rxbuf[4] >> 4);
  iUint=((rxbuf[4] & 0xF) << 8) | rxbuf[5];
  
  hmotor->realPosition.f = (((float)pUint) - 32767.0f) * 0.02197332682f;
  hmotor->realPosition.f *= hmotor->posDirectionCorrection;
  hmotor->realPositionOffseted.f = hmotor->realPosition.f - hmotor->posOffsetDeg;
  hmotor->realPositionOffsetedRad.f = hmotor->realPositionOffseted.f * deg2rad;
  hmotor->realVelocityPresent.f = (((float)vUint) - 2047.0f) * 1.40625f;
  hmotor->realVelocityPresent.f *= hmotor->posDirectionCorrection;
  hmotor->realVelocityPresentRad.f = hmotor->realVelocityPresent.f * deg2rad;
  hmotor->realAccelerationRaw.f = (hmotor->realVelocityPresent.f - hmotor->realVelocityPrevious[0].f) / 0.001f;
  hmotor->realVelocityPrevious[0].f = hmotor->realVelocityPresent.f;
  hmotor->realCurrent.f  = (((float)iUint) - 2048.0f) * 0.0293111871f;
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

void AK10_9_CubeMarsFW_MotorStatusMonitor(AK10_9HandleCubaMarsFW* hmotor, uint32_t timeout_ms)
{
  if ((HAL_GetTick() - hmotor->lastReceivedTime) > timeout_ms)
    hmotor->status = AK10_9_Offline;
  else
    hmotor->status = AK10_9_Online;
}

void AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(AK10_9HandleCubaMarsFW* hmotor, \
                                                 float pos_slope, float vel_slope, float iq_slope, \
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
    if (fabs(diff_pos) > 10.0f)//10 Deg
      hmotor->setPos.f += (diff_pos / abs_diff_pos) * pos_slope * loop_duration_ms;
    else
      hmotor->setPos.f = hmotor->goalPos.f;
    /* Velocity smoother */
    if (fabs(diff_vel) > 10.0f)//10 Deg/sec
      hmotor->setVel.f += (diff_vel / abs_diff_vel) * vel_slope * loop_duration_ms;
    else
      hmotor->setVel.f = hmotor->goalVel.f;
    /* Iq smoother */
    if (fabs(diff_iq) > 1.0f)//1 Deg/sec
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

    AK10_9_MITModeControl_Deg(hmotor, hmotor->setPos.f, hmotor->setVel.f, hmotor->setKp.f, hmotor->setKd.f, hmotor->setIq.f);
  }
  else if (hmotor->enablingStatus == AK10_9_MITMODE_DISABLED)
    AK10_9_MITMode_DisableMotor(hmotor);
}

void AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(AK10_9HandleCubaMarsFW* hmotor, float goal_pos, float goal_vel, \
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
  
  if (abs_diff_pos <= 1.0f && abs_diff_vel <= 1.0f && abs_diff_kp <= 1.0f && abs_diff_kd <= 1.0f && abs_diff_iq <= 1.0f)
    hmotor->ifMITModeParameterSmootherWorkFinished = 1;
  else
    hmotor->ifMITModeParameterSmootherWorkFinished = 0;
}

void AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(AK10_9HandleCubaMarsFW* hmotor, float goal_pos, float goal_vel, \
                                                               float goal_kp, float goal_kd, float goal_iq)
{
  AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(hmotor, goal_pos + hmotor->posOffsetDeg, goal_vel, goal_kp, goal_kd, goal_iq);
}

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
  
  uint16_t pInt = FloatToUint(pos, p_min, p_max, 16);
  uint16_t vInt = FloatToUint(vel, v_min, v_max, 12);
  uint16_t kpInt = FloatToUint(kp, kp_min, kp_max, 12);
  uint16_t kdInt = FloatToUint(kd, kd_min, kd_max, 12);
  uint16_t iInt = FloatToUint(iq, i_min, i_max, 12);
  
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
    if (fabs(diff_pos) > 10.0f * deg2rad)//10 Deg
      hmotor->setPos.f += (diff_pos / abs_diff_pos) * pos_slope_deg * loop_duration_ms;
    else
      hmotor->setPos.f = hmotor->goalPos.f;
    /* Velocity smoother */
    if (fabs(diff_vel) > 10.0f * deg2rad)//10 Deg/sec
      hmotor->setVel.f += (diff_vel / abs_diff_vel) * vel_slope_deg * loop_duration_ms;
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
    AK10_9_DMFW_DisableMotor(hmotor);
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
  
uint16_t FloatToUint(float x, float x_min, float x_max, uint16_t bits)
{
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (uint16_t)((x - x_min) * ((float)((1<<bits)/span)));
}

float UintToFloat(uint16_t x_int, float x_min, float x_max, uint16_t bits)
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
  
  hmotor->realPositionRad.f = UintToFloat(pUint, p_min,p_max, 16);
  hmotor->realPositionRad.f *= hmotor->posDirectionCorrection;
  hmotor->realPositionDeg.f = hmotor->realPositionRad.f * 180.0f / pi;
  hmotor->realPositionOffseted.f = hmotor->realPositionDeg.f - hmotor->posOffsetDeg;
  hmotor->realPositionOffsetedRad.f = hmotor->realPositionOffseted.f * deg2rad;
  hmotor->realVelocityPresentRad.f = UintToFloat(vUint, v_min, v_max, 12);
  hmotor->realVelocityPresentRad.f *= hmotor->posDirectionCorrection;
  hmotor->realVelocityPresent.f = hmotor->realVelocityPresentRad.f * rad2deg;
  hmotor->realAccelerationRaw.f = (hmotor->realVelocityPresent.f - hmotor->realVelocityPrevious[0].f) / 0.001f;
  hmotor->realVelocityPrevious[0].f = hmotor->realVelocityPresent.f;
  hmotor->realCurrent.f  = UintToFloat(iUint, i_min, i_max, 12);
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

/*Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump */
#ifdef DONT_USE_DUMP
void AK10_9_MITMode_PositionSpeedControlCustomized_Deg(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  float setPosIncrement = speed * loop_duration;
  if (hmotor->ifCustomizedPositionSpeedControlStarted)
  {
    if (fabs((double)(hmotor->setPosition.f - position)) >= 1.0f)
    {
      hmotor->ifCustomizedPositionSpeedControlFinished = 0;
      if (position > hmotor->setPosition.f)
        AK10_9_MITModeControl_Deg(hmotor, hmotor->setPosition.f + setPosIncrement, 360.0f, kp, kd, 0.0f);
      else
        AK10_9_MITModeControl_Deg(hmotor, hmotor->setPosition.f - setPosIncrement, 360.0f, kp, kd, 0.0f);
    }
    else
    {
      AK10_9_MITModeControl_Deg(hmotor, position, 360.0f, kp, kd, 0.0f);
      hmotor->ifCustomizedPositionSpeedControlFinished = 1;
    }
  }
  else
  {
    hmotor->setPosition.f = hmotor->realPosition.f;
    hmotor->ifCustomizedPositionSpeedControlStarted = 1;
  }
}

void AK10_9_MITMode_PositionSpeedControlCustomizedWithOffset_Deg(AK10_9HandleCubaMarsFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  AK10_9_MITMode_PositionSpeedControlCustomized_Deg(hmotor, position + hmotor->posOffsetDeg, \
                                                  speed, kp, kd, loop_duration);
}

void AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Rad(AK10_9HandleDMFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  float setPosIncrement = speed * loop_duration;
  if (fabs((double)(hmotor->setPosition.f - position)) >= deg2rad)
  {
    hmotor->ifCustomizedPositionSpeedControlFinished = 0;
    if (position > hmotor->setPosition.f)
      AK10_9_DMFW_MITModeControl_Rad(hmotor, hmotor->setPosition.f + setPosIncrement, 0.0f, kp, kd, 0.0f);
    else
      AK10_9_DMFW_MITModeControl_Rad(hmotor, hmotor->setPosition.f - setPosIncrement, 0.0f, kp, kd, 0.0f);
  }
  else
  {
    AK10_9_DMFW_MITModeControl_Rad(hmotor, position, 0.0f, kp, kd, 0.0f);
    hmotor->ifCustomizedPositionSpeedControlFinished = 1;
  }
}

void AK10_9_DMFW_MITMode_PositionSpeedControlCustomizedWithOffset_Rad(AK10_9HandleDMFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Rad(hmotor, position + hmotor->posOffsetRad, \
                                                  speed, kp, kd, loop_duration);
}

void AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Deg(AK10_9HandleDMFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  float position_rad, speed_rad;
  position_rad = position * deg2rad;
  speed_rad = speed * deg2rad;
  AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Rad(hmotor, position_rad, speed_rad, kp, kd, loop_duration);
}

void AK10_9_DMFW_MITMode_PositionSpeedControlCustomizedWithOffset_Deg(AK10_9HandleDMFW* hmotor, float position, float speed, float kp, float kd, float loop_duration)
{
  AK10_9_DMFW_MITMode_PositionSpeedControlCustomized_Deg(hmotor, position + hmotor->posOffsetDeg, speed, kp, kd, loop_duration);
}
#endif
/*Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump Dump */
