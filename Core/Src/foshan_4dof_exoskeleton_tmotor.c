#include "foshan_4dof_exoskeleton_tmotor.h"

Foshan4DOFExoskeletonTMotorHandle hExoskeletonFoshan4DOFTMotor;

void Foshan4DOFExoskeletonTMotor_Init(float controlLoopPeriod)
{
	hExoskeletonFoshan4DOFTMotor.controlLoopPeriod = controlLoopPeriod;
	
	hExoskeletonFoshan4DOFTMotor.hMotorRightKnee = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_MOTOR, 1.1457f, -1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
  hExoskeletonFoshan4DOFTMotor.hMotorRightHip = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_MOTOR, 1.2339f, -1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
	hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_LEFT_KNEE_MOTOR, 1.1457f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
  hExoskeletonFoshan4DOFTMotor.hMotorLeftHip = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_LEFT_HIP_MOTOR, 1.2339f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
	
	hExoskeletonFoshan4DOFTMotor.gravLeftCoef_1 = 0.0f;
	hExoskeletonFoshan4DOFTMotor.gravLeftCoef_2 = 0.0f;
	hExoskeletonFoshan4DOFTMotor.gravRightCoef_1 = 0.0f;
	hExoskeletonFoshan4DOFTMotor.gravRightCoef_2 = 0.0f;
	
  
  hExoskeletonFoshan4DOFTMotor.motorCmdSequence = FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED;
	hExoskeletonFoshan4DOFTMotor.task = FOSHAN_4DOF_EXO_TASK_IDLE;
}

void Foshan4DOFExoskeletonTMotor_CenterControl(void)
{
	switch (hExoskeletonFoshan4DOFTMotor.task)
  {
    case FOSHAN_4DOF_EXO_TASK_OFF:
      break;
  	case FOSHAN_4DOF_EXO_TASK_IDLE:
    {
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED)
        hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip, 0.0f);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE)
				AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee, 0.0f);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP)
        AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip, 0.0f);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE)
				AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee, 0.0f);
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_ZEROING:
    {
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITMode_Zeroing(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE)
				AK10_9_MITMode_Zeroing(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP)
        AK10_9_MITMode_Zeroing(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE)
      {
				AK10_9_MITMode_Zeroing(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee);
        hExoskeletonFoshan4DOFTMotor.task = FOSHAN_4DOF_EXO_TASK_IDLE;
      }
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
      break;
    }
  	case FOSHAN_4DOF_EXO_TASK_MANUAL_CONTROL:
    {
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_GRAVITY_COMPENSATION:
    {
			Foshan4DOFExoskeletonTMotor_CalculateGravityCompensation(&hExoskeletonFoshan4DOFTMotor);
			
			if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED)
        hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip, \
      hExoskeletonFoshan4DOFTMotor.calGravLeftHip * hExoskeletonFoshan4DOFTMotor.hMotorLeftHip.posDirectionCorrection / \
      hExoskeletonFoshan4DOFTMotor.hMotorLeftHip.kt);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE)
				AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee, \
      hExoskeletonFoshan4DOFTMotor.calGravLeftKnee * hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee.posDirectionCorrection / \
      hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee.kt);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP)
        AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip, \
      -hExoskeletonFoshan4DOFTMotor.calGravRightHip * hExoskeletonFoshan4DOFTMotor.hMotorRightHip.posDirectionCorrection / \
      hExoskeletonFoshan4DOFTMotor.hMotorRightHip.kt);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE)
				AK10_9_MITModeCurrentControl(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee, \
      -hExoskeletonFoshan4DOFTMotor.calGravRightKnee * hExoskeletonFoshan4DOFTMotor.hMotorRightKnee.posDirectionCorrection / \
      hExoskeletonFoshan4DOFTMotor.hMotorRightKnee.kt);
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_ENABLE_MOTORS:
    {
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITMode_EnableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE)
				AK10_9_MITMode_EnableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP)
        AK10_9_MITMode_EnableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE)
      {
				AK10_9_MITMode_EnableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee);
        hExoskeletonFoshan4DOFTMotor.task = FOSHAN_4DOF_EXO_TASK_IDLE;
      }
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_DISABLE_MOTORS:
    {
			if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITMode_DisableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE)
				AK10_9_MITMode_DisableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP)
        AK10_9_MITMode_DisableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip);
			else if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE)
      {
				AK10_9_MITMode_DisableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee);
        hExoskeletonFoshan4DOFTMotor.task = FOSHAN_4DOF_EXO_TASK_OFF;
      }
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
  		break;
    }
  	default:
  		break;
  }
	
	
	
	
	
	if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE + 1)
		hExoskeletonFoshan4DOFTMotor.motorCmdSequence = FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED;
	
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee, 300);
}

void Foshan4DOFExoskeletonTMotor_CalculateGravityCompensation(Foshan4DOFExoskeletonTMotorHandle* hexo)
{
	hexo->calGravLeftKnee = hexo->gravLeftCoef_2 * sinf((hexo->hMotorLeftKnee.realPosition.f + hexo->hMotorLeftHip.realPosition.f) * deg2rad);
	hexo->calGravRightKnee = hexo->gravRightCoef_2 * sinf((hexo->hMotorRightKnee.realPosition.f + hexo->hMotorRightHip.realPosition.f)* deg2rad);
	hexo->calGravLeftHip = hexo->calGravLeftKnee            + hexo->gravLeftCoef_1 * sinf(hexo->hMotorLeftHip.realPosition.f * deg2rad);
	hexo->calGravRightHip = hexo->calGravRightKnee          + hexo->gravRightCoef_1 * sinf(hexo->hMotorRightHip.realPosition.f * deg2rad);
}
