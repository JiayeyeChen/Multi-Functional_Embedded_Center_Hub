#include "exoskeleton.h"
#include "ak10-9_v2_testing.h"

BNO055Handle hIMURightThigh, hIMURightKnee;
Exoskeleton_SystemIDHandle hSystemID;
Exoskeleton_GravityCompensation hGravityCompensation;
ExoskeletonHandle hExoskeleton;
Exoskeleton_MuscularTorqueEstimationHandle hMuscularTorque;
Exoskeleton_AugmentedControlHandle hAugmentedControl;

void EXOSKELETON_Init(void)
{
  hIMURightThigh.hcan = &hcan2;
  hIMURightThigh.operationMode = 0xFF;
  hIMURightThigh.operationModeENUM = IMU_MODE_ACCONLY;
  hIMURightThigh.CANID_SET_MODE_NDOF = CAN_ID_IMU_GET_DATA_NDOF_RIGHT_THIGH;
  hIMURightThigh.CANID_SET_MODE_GYROONLY = CAN_ID_IMU_GET_DATA_GYROONLY_RIGHT_THIGH;
  hIMURightThigh.CANID_SET_MODE_ACCONLY = CAN_ID_IMU_GET_DATA_ACCONLY_RIGHT_THIGH;
  hIMURightThigh.lpfCutOffFrequency = 10.0f;
  hIMURightThigh.lpfDuration = 0.002f;
  hIMURightThigh.lpfAlpha = 2.0f * pi * hIMURightThigh.lpfCutOffFrequency * hIMURightThigh.lpfDuration / (1.0f + 2.0f * pi * hIMURightThigh.lpfCutOffFrequency * hIMURightThigh.lpfDuration);
  
  hMuscularTorque.ifEstimating = 0;
  hMuscularTorque.muscularTorqueHip.f = 0.0f;
  hMuscularTorque.muscularTorqueKnee.f = 0.0f;
  
  hSystemID.sysIDResults_J1.f = 1.29053f;
  hSystemID.sysIDResults_X1.f = 4.21455f;
  hSystemID.sysIDResults_J2.f = 0.247732f;
  hSystemID.sysIDResults_X2.f = 1.13107f;
  
  hAugmentedControl.hipJointAugmentedControlThrottle = 0.0f;
  hAugmentedControl.kneeJointAugmentedControlThrottle = 0.0f;
  hAugmentedControl.ifAugmentedControl = 0;
  
  hExoskeleton.hgravitycompensation = &hGravityCompensation;
  hExoskeleton.hsysid = &hSystemID;
  hExoskeleton.mainTask = EXOSKELETON_MAIN_TASK_FREE;
  hExoskeleton.hmusculartorque = &hMuscularTorque;
  hExoskeleton.L1.f = 0.0f;
  hExoskeleton.haugmentedcontrol = &hAugmentedControl;
}

void EXOSKELETON_Set_Common_Datalog_Label(void)
{
  USB_SendDataSlotLabel("11", "theta0", "theta1", "theta1 vel", \
                        "theta1 acc", "theta2", "theta2 vel", "theta2 acc", "torque1", "torque2", "HipMTq", "KneeMTq");
}

void EXOSKELETON_CommonDatalogManager(void)
{
  EXOSKELETON_UpdateCommonDataSlot();
  USB_DataLogManager(EXOSKELETON_Set_Common_Datalog_Label, dataSlots_Exoskeleton_Common);
}

void EXOSKELETON_UpdateCommonDataSlot(void)
{
  uint8_t ptr = 0;
  dataSlots_Exoskeleton_Common[ptr++].f = 0.0f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightHip.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightHip.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightHip.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightKnee.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightKnee.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightKnee.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightHip.realTorque.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hAKMotorRightKnee.realTorque.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hExoskeleton.hmusculartorque->muscularTorqueHip.f;
  dataSlots_Exoskeleton_Common[ptr++].f = hExoskeleton.hmusculartorque->muscularTorqueKnee.f;
  hUSB.ifNewDataLogPiece2Send = 1;
}

void EXOSKELETON_SystemID_Init(void)
{
  hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_FREE;
  hSystemID.hipProfilingAmp = 0.0f;
  hSystemID.hipProfilingFreq = 0.0f;
  hSystemID.kneeProfilingAmp = 0.0f;
  hSystemID.kneeProfilingFreq = 0.0f;
  hSystemID.ifIdentified = 0;
}

void EXOSKELETON_GetIMUFeedbackLiAcc(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.liaccX.b8[0] = data[0];
  himu->rawData.liaccX.b8[1] = data[1];
  himu->rawData.liaccY.b8[0] = data[2];
  himu->rawData.liaccY.b8[1] = data[3];
  himu->rawData.liaccZ.b8[0] = data[4];
  himu->rawData.liaccZ.b8[1] = data[5];
}

void EXOSKELETON_GetIMUFeedbackGyro(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.gyroX.b8[0] = data[0];
  himu->rawData.gyroX.b8[1] = data[1];
  himu->rawData.gyroY.b8[0] = data[2];
  himu->rawData.gyroY.b8[1] = data[3];
  himu->rawData.gyroZ.b8[0] = data[4];
  himu->rawData.gyroZ.b8[1] = data[5];
}
void EXOSKELETON_GetIMUFeedbackQuaternion(BNO055Handle* himu, uint8_t data[])
{
  
}

void EXOSKELETON_GetIMUFeedbackStatus(BNO055Handle* himu, uint8_t data[])
{
  himu->calibStatus = data[0];
  himu->operationMode = data[1];
  himu->deadCount = data[2];
}

void EXOSKELETON_SetIMUMode_9_DOF(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_NDOF;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}
void EXOSKELETON_SetIMUMode_GYRO_Only(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_GYROONLY;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}

void EXOSKELETON_SetIMUMode_ACC_Only(BNO055Handle* himu)
{
  himu->txHeader.StdId = himu->CANID_SET_MODE_ACCONLY;
  himu->txHeader.RTR = 0;
  himu->txHeader.IDE = 0;
  himu->txHeader.DLC = 1;
  HAL_CAN_AddTxMessage(himu->hcan, &himu->txHeader, himu->txBuf, himu->pTxMailbox);
}

void EXOSKELETON_GetIMUFeedbackAcc(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.AccX.b8[0] = data[0];
  himu->rawData.AccX.b8[1] = data[1];
  himu->rawData.AccY.b8[0] = data[2];
  himu->rawData.AccY.b8[1] = data[3];
  himu->rawData.AccZ.b8[0] = data[4];
  himu->rawData.AccZ.b8[1] = data[5];
  himu->parsedData.AccX.f = (1.0f - himu->lpfAlpha) * himu->lpfAccXFilteredPrevious + himu->lpfAlpha * ((float)himu->rawData.AccX.b16) / 100.0f;
  himu->parsedData.AccY.f = (1.0f - himu->lpfAlpha) * himu->lpfAccYFilteredPrevious + himu->lpfAlpha * ((float)himu->rawData.AccY.b16) / 100.0f;
  himu->parsedData.AccZ.f = (1.0f - himu->lpfAlpha) * himu->lpfAccZFilteredPrevious + himu->lpfAlpha * ((float)himu->rawData.AccZ.b16) / 100.0f;
  himu->lpfAccXFilteredPrevious = himu->parsedData.AccX.f;
  himu->lpfAccYFilteredPrevious = himu->parsedData.AccY.f;
  himu->lpfAccZFilteredPrevious = himu->parsedData.AccZ.f;
}
void EXOSKELETON_GetIMUFeedbackMag(BNO055Handle* himu, uint8_t data[])
{
  himu->rawData.MagX.b8[0] = data[0];
  himu->rawData.MagX.b8[1] = data[1];
  himu->rawData.MagY.b8[0] = data[2];
  himu->rawData.MagY.b8[1] = data[3];
  himu->rawData.MagZ.b8[0] = data[4];
  himu->rawData.MagZ.b8[1] = data[5];
}

void EXOSKELETON_SystemIDManager(void)
{
  static uint32_t enterProfilingTimeStamp;

  USB_DatalogCargoReceiveManager(EXOSKELETON_SystemID_Set_Datalog_Label);
  
  switch (hSystemID.curTask)
  {
  	case EXOSKELETON_SYSTEMID_TASK_FREE:
  		break;
  	case EXOSKELETON_SYSTEMID_TASK_START:
      USB_SendText("System ID start request");
      if(hUSB.ifNewCargo)
      {
        char rplyMsg[] = "SystemID start noted";
        if (USB_CompareRxCfmMsgWithStr(rplyMsg, sizeof(rplyMsg) - 1))
        {
////////////          /* For testing*/
////////////          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS;
////////////          break;
////////////          ////////////////
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START;
          USB_SetNewDataSlotLen(sizeof(dataSlots_Exoskeleton_SystemID)/4);
          USB_DataLogStart();
        }
        hUSB.ifNewCargo = 0;
      }
  		break;
    case EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_WAIT_FOR_START:
      if (hUSB.datalogTask == DATALOG_TASK_START)
        USB_DataLogStart();
      break;
    case EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_POSITIONING:
      
      AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, \
                                                                SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT, \
                                                                0.0f, 200.0f, 2.0f, 0.0f);
      AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                          SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT, \
                                                          0.0f, 200.0f, 2.0f, 0.0f);
//////////    /* for debug */
//////////    hAKMotorRightHip.ifCustomizedPositionSpeedControlFinished = 1;
//////////    hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////    ///////////////
      if (hAKMotorRightHip.ifMITModeParameterSmootherWorkFinished && hAKMotorRightKnee.ifMITModeParameterSmootherWorkFinished)
      {
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING;
        enterProfilingTimeStamp = HAL_GetTick();
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING:
      if (HAL_GetTick() - enterProfilingTimeStamp <= SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME)
      {
        
        AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, \
                                                                SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT, \
                                                                0.0f, 200.0f, 2.0f, 0.0f);
        AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                          SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT, \
                                                          0.0f, 200.0f, 2.0f, 0.0f);
      }
      else
      {
        if (HAL_GetTick() - enterProfilingTimeStamp - SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME <= hSystemID.kneeProfilingTime)
        {
          EXOSKELETON_SystemID_UpdateDataSlot();
          USB_DataLogSingleCargoTransmit(dataSlots_Exoskeleton_SystemID);
          
          AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                          SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT, \
                                                          0.0f, 200.0f, 2.0f, 0.0f);
          EXOSKELETON_SystemID_KneeJoint_MotorProfilingSinWave(&hAKMotorRightKnee, hSystemID.kneeProfilingAmp, \
                                                               hSystemID.kneeProfilingFreq, enterProfilingTimeStamp + SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME);
        }
        else
        {
          USB_DataLogEnd();
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START;

        }
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START:
      if (hUSB.datalogTask == DATALOG_TASK_END)
        USB_DataLogEnd();
      if (hUSB.datalogTask == DATALOG_TASK_FREE)
        USB_DataLogStart();
      if (hUSB.datalogTask == DATALOG_TASK_START)
        USB_DataLogStart();
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_POSITIONING:
      
      AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, \
                                                                SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT, \
                                                                0.0f, 200.0f, 2.0f, 0.0f);
    #ifdef HIP_JOINT_LEARNING_USE_CURRENT_CONTROL
      AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                          0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    #else
      AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                          SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT, \
                                                          0.0f, 200.0f, 2.0f, 0.0f);
    #endif
//////////      /* for debug */
//////////      hAKMotorRightHip.ifCustomizedPositionSpeedControlFinished = 1;
//////////      hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////      ///////////////
      if (hAKMotorRightHip.ifMITModeParameterSmootherWorkFinished && hAKMotorRightKnee.ifMITModeParameterSmootherWorkFinished)
      {
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING;
        enterProfilingTimeStamp = HAL_GetTick();
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING:
      if (HAL_GetTick() - enterProfilingTimeStamp <= SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME)
      {
        
        AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, \
                                                                  SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT, \
                                                                  0.0f, 200.0f, 2.0f, 0.0f);
        #ifdef HIP_JOINT_LEARNING_USE_CURRENT_CONTROL
          AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                              0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        #else
          AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, \
                                                              SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT, \
                                                              0.0f, 200.0f, 2.0f, 0.0f);
        #endif
      }
      else
      {
        if (HAL_GetTick() - enterProfilingTimeStamp - SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME <= hSystemID.hipProfilingTime)
        {
          EXOSKELETON_SystemID_UpdateDataSlot();
          USB_DataLogSingleCargoTransmit(dataSlots_Exoskeleton_SystemID);
          
          AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, \
                                                                  SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT, \
                                                                  0.0f, 200.0f, 2.0f, 0.0f);
          #ifdef HIP_JOINT_LEARNING_USE_CURRENT_CONTROL
          EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_CurrentControl(&hAKMotorRightHip,  hSystemID.hipProfilingAmp, \
                                                               hSystemID.hipProfilingFreq, enterProfilingTimeStamp + SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME);
          #else
          EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_PositionControl(&hAKMotorRightHip, hSystemID.hipProfilingAmp, \
                                                               hSystemID.hipProfilingFreq, enterProfilingTimeStamp + SYSTEMID_JOINT_POSITIONING_STABILIZING_TIME);
          #endif
        }
        else
        {
          USB_DataLogEnd();
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS;
        }
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS:
      /*Knee joint goes to 10 deg. Hip joint goes to 180 deg.*/
      AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, 10.0f, 0.0f, 100.0f, 2.0f, 0.0f);
      #ifdef HIP_JOINT_LEARNING_USE_CURRENT_CONTROL
      AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      #else
      AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightHip, 180.0f, 0.0f, 100.0f, 2.0f, 0.0f);
      #endif
//////////      /* for debug */
//////////      hAKMotorRightHip.ifCustomizedPositionSpeedControlFinished = 1;
//////////      hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////      ///////////////
      if (hAKMotorRightHip.ifMITModeParameterSmootherWorkFinished && hAKMotorRightKnee.ifMITModeParameterSmootherWorkFinished)
      {
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS;
        AK10_9_DMFW_MITMode_ZeroingControlParameters(&hAKMotorRightHip);
        AK10_9_CubeMarsFW_MITMode_ZeroingControlParameters(&hAKMotorRightKnee);
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS:
      USB_SendText("Receiving results from PC...");
      if(hUSB.ifNewCargo)
      {
        char expMsg[] = "LSA";
        if (!memcmp(expMsg, hUSB.rxMessageCfrm, 3))
        {
          uint8_t ptr = 3;
          hSystemID.sysIDResults_J1.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J1.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J1.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J1.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X1.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X1.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X1.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X1.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J2.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J2.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J2.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_J2.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X2.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X2.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X2.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_X2.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.ifIdentified = 1;
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_END;
          hUSB.ifNewCargo = 0;
        }
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_END:
      USB_SendText("System ID end");
      if(hUSB.ifNewCargo)
      {
        char rplyMsg[] = "Roger that";
        if (USB_CompareRxCfmMsgWithStr(rplyMsg, sizeof(rplyMsg) - 1))
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_FREE;
        hUSB.ifNewCargo = 0;
      }
      break;
  	default:
  		break;
  }
}

void EXOSKELETON_SystemID_KneeJoint_MotorProfilingSinWave\
     (AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift)
     {
       float t = (float)(HAL_GetTick() - time_stamp_shift) / 1000.0f;
       float motor_profiling_trajectory = SYSTEMID_KNEE_JOINT_LEARNING_STARTING_POSITION_KNEE_JOINT - amplitude / 2.0f + (amplitude / 2.0f) * (float)cos(fre * 2.0f * pi * t);
       AK10_9_CubaMarsFW_MITMode_ContinuousControlWithOffset_Deg(&hAKMotorRightKnee, motor_profiling_trajectory, 0.0f, 200.0f, 3.0f, 0.0f);
     }
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_PositionControl\
     (AK10_9HandleDMFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift)
     {
       float t = (float)(HAL_GetTick() - time_stamp_shift) / 1000.0f;
       float motor_profiling_trajectory = SYSTEMID_HIP_JOINT_LEARNING_STARTING_POSITION_HIP_JOINT - \
                                          amplitude / 2.0f + (amplitude / 2.0f) * (float)cos(fre * 2.0f * pi * t);
       AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(hmotor, motor_profiling_trajectory, 0.0f, 499.0f, 3.0f, 0.0f);
     }
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_CurrentControl \
     (AK10_9HandleDMFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift)
     {
       float t = (float)(HAL_GetTick() - time_stamp_shift) / 1000.0f;
       float motor_profiling_trajectory = -(amplitude / 2.0f) + (amplitude / 2.0f) * (float)cos(fre * 2.0f * pi * t);
       AK10_9_DMFW_MITMode_ContinuousControlWithOffset_Deg(hmotor, 0.0f, 0.0f, 0.0f, 0.0f, motor_profiling_trajectory);
     }

void EXOSKELETON_SystemID_Set_Datalog_Label(void)
{
  USB_SendDataSlotLabel("9", "theta0", "theta1", "theta1 vel", \
                        "theta1 acc", "theta2", "theta2 vel", "theta2 acc", "torque1", "torque2");
}

void EXOSKELETON_SystemID_UpdateDataSlot(void)
{
  uint8_t ptr = 0;
  dataSlots_Exoskeleton_SystemID[ptr++].f = 0.0f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip.realTorque.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realTorque.f;
}

void EXOSKELETON_GravityCompensation_Init(Exoskeleton_GravityCompensation* hgravitycompensation)
{
  hgravitycompensation->ifGravityCompensationStarted = 0;
  hgravitycompensation->throttleHip = 0.0f;
  hgravitycompensation->throttleKnee = 0.0f;
  hgravitycompensation->torqueDesiredHip.f = 0.0f;
  hgravitycompensation->torqueDesiredKnee.f = 0.0f;
}

void EXOSKELETON_CentreControl(void)
{
  switch (hExoskeleton.mainTask)
  {
    case EXOSKELETON_MAIN_TASK_SYSTEM_ID:
     if (hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING || \
         hSystemID.curTask == EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING)
      {
        AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(&hAKMotorRightKnee, \
                                                           180.0f, 180.0f, 1.0f, 200.0f, 2.5f, 0.001f);
        AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
                                                     180.0f, 180.0f, 1.0f, 200.0f, 2.5f, 0.001f);
      }
      else
      {
        AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(&hAKMotorRightKnee, \
                                                           20.0f, 60.0f, 1.0f, 200.0f, 2.5f, 0.001f);
        AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
                                                     20.0f, 60.0f, 30.0f, 200.0f, 2.5f, 0.001f);
      }
      break;
    case EXOSKELETON_MAIN_TASK_GRAVITY_COMPENSATION:
      AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(&hAKMotorRightKnee, \
                                                         0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.001f);
      AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
                                                   0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.001f);
      break;
    case EXOSKELETON_MAIN_TASK_FREE:
      AK10_9_MITModeControl_Deg(&hAKMotorRightKnee, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      AK10_9_DMFW_MITModeControl_Rad(&hAKMotorRightHip, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
      break;
    case EXOSKELETON_MAIN_TASK_AUGMENTED_CONTROL:
      AK10_9_CubeMarsFW_MITMode_ContinuousControlManager(&hAKMotorRightKnee, \
                                                         0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.001f);
      AK10_9_DMFW_MITMode_ContinuousControlManager(&hAKMotorRightHip, \
                                                   0.0f, 0.0f, 5.0f, 0.0f, 0.0f, 0.001f);
      break;
    default:
      break;
  }
  
//  EXOSKELETON_SystemIDManager();
  EXOSKELETON_GravityCompemsationManager();
  EXOSKELETON_MuscularTorqueCalculation(&hExoskeleton);
  EXOSKELETON_AugmentedControlManager();
}

void EXOSKELETON_GravityCompemsationManager(void)
{
  hGravityCompensation.torqueDesiredHip.f = -GRAVITATIONAL_ACCELERATION * (hSystemID.sysIDResults_X1.f * \
                                            sin(hAKMotorRightHip.realPositionOffsetedRad.f) + hSystemID.sysIDResults_X2.f * \
                                            sin(hAKMotorRightHip.realPositionOffsetedRad.f + hAKMotorRightKnee.realPositionOffsetedRad.f));
  hGravityCompensation.torqueDesiredHip.f *= hGravityCompensation.throttleHip;
  hGravityCompensation.torqueDesiredKnee.f = -GRAVITATIONAL_ACCELERATION * (hSystemID.sysIDResults_X2.f * \
                                             sin(hAKMotorRightHip.realPositionOffsetedRad.f + \
                                             hAKMotorRightKnee.realPositionOffsetedRad.f));
  hGravityCompensation.torqueDesiredKnee.f *= hGravityCompensation.throttleKnee;
  
  if (hGravityCompensation.ifGravityCompensationStarted)
  {
    AK10_9_DMFW_MITMode_ContinuousControl_Rad(&hAKMotorRightHip, 0.0f, 0.0f, 0.0f, 0.0f, \
                                               hGravityCompensation.torqueDesiredHip.f / hAKMotorRightHip.kt);
    AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(&hAKMotorRightKnee, 0.0f, 0.0f, 0.0f, 0.0f, \
                                                    hGravityCompensation.torqueDesiredKnee.f / hAKMotorRightKnee.kt);
  }
}

void EXOSKELETON_MuscularTorqueCalculation(ExoskeletonHandle* hexoskeleton)
{
  if (hexoskeleton->hmusculartorque->ifEstimating)
  {
    float M11, M12, M22, C1, C2, G1, G2;
    M11 = hexoskeleton->hsysid->sysIDResults_J1.f + 2.0f * hexoskeleton->hsysid->sysIDResults_X2.f * \
          hexoskeleton->L1.f * cos(hAKMotorRightKnee.realPositionOffsetedRad.f);
    
    M12 = hexoskeleton->hsysid->sysIDResults_J2.f + hexoskeleton->hsysid->sysIDResults_X2.f * \
          hexoskeleton->L1.f * cos(hAKMotorRightKnee.realPositionOffsetedRad.f);
    
    M22 = hexoskeleton->hsysid->sysIDResults_J2.f;
    
    C1 = -hAKMotorRightKnee.realVelocityPresentRad.f * (2.0f * hAKMotorRightHip.realVelocityPresentRad.f + \
         hAKMotorRightKnee.realVelocityPresentRad.f) * hexoskeleton->hsysid->sysIDResults_X2.f * \
         hexoskeleton->L1.f * sin(hAKMotorRightKnee.realPositionOffsetedRad.f);
    
    C2 = powf(hAKMotorRightHip.realPositionOffsetedRad.f, 2.0f) * hexoskeleton->hsysid->sysIDResults_X2.f * \
         hexoskeleton->L1.f * sin(hAKMotorRightKnee.realPositionOffsetedRad.f);
    
    G1 = -GRAVITATIONAL_ACCELERATION * (hexoskeleton->hsysid->sysIDResults_X1.f * \
         sin(hAKMotorRightHip.realPositionOffsetedRad.f) + hexoskeleton->hsysid->sysIDResults_X2.f * \
         sin(hAKMotorRightHip.realPositionOffsetedRad.f + hAKMotorRightKnee.realPositionOffsetedRad.f));
    
    G2 = -GRAVITATIONAL_ACCELERATION * hexoskeleton->hsysid->sysIDResults_X2.f * \
         sin(hAKMotorRightHip.realPositionOffsetedRad.f + hAKMotorRightKnee.realPositionOffsetedRad.f);
         
    
    hexoskeleton->hmusculartorque->muscularTorqueHip.f = M11 * hAKMotorRightHip.realAccelerationFilteredRad.f + \
                                                         M12 * hAKMotorRightKnee.realAccelerationFilteredRad.f + \
                                                         C1 + G1 - hAKMotorRightHip.realTorque.f;
    hexoskeleton->hmusculartorque->muscularTorqueKnee.f = M12 * hAKMotorRightHip.realAccelerationFilteredRad.f + \
                                                          M22 * hAKMotorRightKnee.realAccelerationFilteredRad.f + \
                                                          C2 + G2 - hAKMotorRightKnee.realTorque.f;
  }
}

void EXOSKELETON_AugmentedControlManager(void)
{
  if (hExoskeleton.haugmentedcontrol->ifAugmentedControl)
  {
    AK10_9_DMFW_MITMode_ContinuousControl_Rad(&hAKMotorRightHip, 0.0f, 0.0f, 0.0f, 0.0f, \
                                              hExoskeleton.haugmentedcontrol->hipJointAugmentedControlThrottle * \
                                              hExoskeleton.hmusculartorque->muscularTorqueHip.f / hAKMotorRightHip.kt);
    AK10_9_CubaMarsFW_MITMode_ContinuousControl_Deg(&hAKMotorRightKnee, 0.0f, 0.0f, 0.0f, 0.0f, \
                                                    hExoskeleton.haugmentedcontrol->kneeJointAugmentedControlThrottle * \
                                                    hExoskeleton.hmusculartorque->muscularTorqueKnee.f/ hAKMotorRightKnee.kt);
  }
}
