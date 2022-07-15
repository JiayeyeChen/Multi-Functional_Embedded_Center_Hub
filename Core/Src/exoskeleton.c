#include "exoskeleton.h"
#include "ak10-9_v2_testing.h"

BNO055Handle hIMURightThigh, hIMURightKnee;
Exoskeleton_SystemIDHandle hSystemID;

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
  static float temJointPosition;
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
      
      /* for hip position control*/
        AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightHip_old, 180.0f, 36.0f, 0.002f);
      /* for hip current control*/
//      AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
      ////////////////////////////
      
      AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightKnee, 90.0f, 36.0f, 0.002f);
//////////    /* for debug */
//////////    hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished = 1;
//////////    hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////    ///////////////
      if (hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished && hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished)
      {
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING;
        enterProfilingTimeStamp = HAL_GetTick();
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_KNEE_JOINT_MOVEMENT_ONGOING:
      if (HAL_GetTick() - enterProfilingTimeStamp <= 3000)
      {
        /* for hip position control*/
        AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightHip_old, 180.0f);
        /* for hip current control*/
//        AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
        ////////////////////////////
        AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightKnee, 90.0f);
      }
      else
      {
        if (HAL_GetTick() - enterProfilingTimeStamp - 3000 <= hSystemID.kneeProfilingTime)
        {
          EXOSKELETON_SystemID_UpdateDataSlot();
          USB_DataLogSingleCargoTransmit(dataSlots_Exoskeleton_SystemID);
          /* for hip position control*/
          AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightHip_old, 180.0f);
          /* for hip current control */
//          AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
          /////////////////////////////
          EXOSKELETON_SystemID_KneeJoint_MotorProfilingSinWave(&hAKMotorRightKnee, hSystemID.kneeProfilingAmp, \
                                                               hSystemID.kneeProfilingFreq, enterProfilingTimeStamp + 3000);
        }
        else
        {
          USB_DataLogEnd();
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START;
          temJointPosition = hAKMotorRightKnee.realPositionOffseted.f;
        }
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_WAIT_FOR_START:
      /* for hip position control*/
      AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightHip_old, 180.0f);
      /* for hip current control*/
//      AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
      ////////////////////////////
      AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightKnee, temJointPosition);
      if (hUSB.datalogTask == DATALOG_TASK_END)
        USB_DataLogEnd();
      if (hUSB.datalogTask == DATALOG_TASK_FREE)
        USB_DataLogStart();
      if (hUSB.datalogTask == DATALOG_TASK_START)
        USB_DataLogStart();
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_POSITIONING:
      /* for hip position control*/
      AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightHip_old, 180.0f, 36.0f, 0.002f);
      /* for hip current control*/
//      AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
      ////////////////////////////
      AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightKnee, 90.0f, 36.0f, 0.002f);
    
    
//////////      /* for debug */
//////////      hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished = 1;
//////////      hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////      ///////////////
      
      /* for hip position control*/
      if (hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished && hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished)
      {
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING;
        enterProfilingTimeStamp = HAL_GetTick();
      }
      /* for hip current control*/
////      if (hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished)
////      {
////        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING;
////        enterProfilingTimeStamp = HAL_GetTick();
////      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_HIP_JOINT_MOVEMENT_ONGOING:
      if (HAL_GetTick() - enterProfilingTimeStamp <= 3000)
      {
        /* for hip position control*/
        AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightHip_old, 180.0f);
        /* for hip current control*/
//        AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
        ////////////////////////////
        AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightKnee, 90.0f);
      }
      else
      {
        if (HAL_GetTick() - enterProfilingTimeStamp - 3000 <= hSystemID.hipProfilingTime)
        {
          EXOSKELETON_SystemID_UpdateDataSlot();
          USB_DataLogSingleCargoTransmit(dataSlots_Exoskeleton_SystemID);
          AK10_9_ServoMode_PositionControlWithOffset(&hAKMotorRightKnee, 90.0f);
          /* for hip position control*/
          EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_PositionControl(&hAKMotorRightHip_old, hSystemID.hipProfilingAmp, \
                                                               hSystemID.hipProfilingFreq, enterProfilingTimeStamp + 3000);
          /* for hip current control*/
//          EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_CurrentControl(&hAKMotorRightHip_old, hSystemID.hipProfilingAmp, \
//                                                       hSystemID.hipProfilingFreq, enterProfilingTimeStamp + 3000);
        }
        else
        {
          USB_DataLogEnd();
          hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS;
        }
      }
      break;
    case EXOSKELETON_SYSTEMID_TASK_RELEASING_JOINTS:
      /* for hip position control*/
      AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightHip_old, 180.0f, 36.0f, 0.002f);
      /* for hip current control*/
//      AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, 0.0f);
      ////////////////////////////
      AK10_9_ServoMode_PositionSpeedControlCustomizedWithOffset(&hAKMotorRightKnee, 5.0f, 36.0f, 0.002f);
//////////      /* for debug */
//////////      hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished = 1;
//////////      hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished = 1;
//////////      ///////////////
      if (hAKMotorRightHip_old.ifCustomizedPositionSpeedControlFinished && hAKMotorRightKnee.ifCustomizedPositionSpeedControlFinished)
        hSystemID.curTask = EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS;
      break;
    case EXOSKELETON_SYSTEMID_TASK_RECEIVING_RESULTS:
      USB_SendText("Receiving results from PC...");
      if(hUSB.ifNewCargo)
      {
        char expMsg[] = "LSA";
        if (!memcmp(expMsg, hUSB.rxMessageCfrm, 3))
        {
          uint8_t ptr = 3;
          hSystemID.sysIDResults_a1.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a1.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a1.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a1.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m1.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m1.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m1.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m1.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a2.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a2.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a2.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_a2.b8[3] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m2.b8[0] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m2.b8[1] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m2.b8[2] = hUSB.rxMessageCfrm[ptr++];
          hSystemID.sysIDResults_m2.b8[3] = hUSB.rxMessageCfrm[ptr++];
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
       float motor_profiling_trajectory = 90.0f - amplitude / 2.0f + (amplitude / 2.0f) * (float)cos(fre * 2.0f * pi * t);
       AK10_9_ServoMode_PositionControlWithOffset(hmotor, motor_profiling_trajectory);
     }
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_PositionControl\
     (AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift)
     {
       float t = (float)(HAL_GetTick() - time_stamp_shift) / 1000.0f;
       float motor_profiling_trajectory = 180.0f + (amplitude / 2.0f) * (float)sin(fre * 2.0f * pi * t);
       AK10_9_ServoMode_PositionControlWithOffset(hmotor, motor_profiling_trajectory);
     }
void EXOSKELETON_SystemID_HipJoint_MotorProfilingSinWave_CurrentControl \
     (AK10_9HandleCubaMarsFW* hmotor, float amplitude, float fre, uint32_t time_stamp_shift)
     {
       float t = (float)(HAL_GetTick() - time_stamp_shift) / 1000.0f;
       float motor_profiling_trajectory = (amplitude / 2.0f) * (float)sin(fre * 2.0f * pi * t);
       AK10_9_ServoMode_CurrentControl(&hAKMotorRightHip_old, motor_profiling_trajectory);
     }

void EXOSKELETON_SystemID_Set_Datalog_Label(void)
{
  USB_SendDataSlotLabel("9", "theta0", "theta1", "theta1 vel", \
                        "theta1 acc", "theta2", "theta2 vel", "theta2 acc", "torque1", "torque2");
}

void EXOSKELETON_SystemID_UpdateDataSlot(void)
{
  uint8_t ptr = 0;
  dataSlots_Exoskeleton_SystemID[ptr++].f = pi/12.0f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip_old.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip_old.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip_old.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realPositionOffsetedRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realVelocityPresentRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realAccelerationFilteredRad.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightHip_old.realTorque.f;
  dataSlots_Exoskeleton_SystemID[ptr++].f = hAKMotorRightKnee.realTorque.f;
}
