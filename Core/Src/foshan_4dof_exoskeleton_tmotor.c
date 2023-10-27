#include "foshan_4dof_exoskeleton_tmotor.h"

Foshan4DOFExoskeletonTMotorHandle hExoskeletonFoshan4DOFTMotor;

void Foshan4DOFExoskeletonTMotor_Init(float controlLoopPeriod)
{
	hExoskeletonFoshan4DOFTMotor.controlLoopPeriod = controlLoopPeriod;
	
	hExoskeletonFoshan4DOFTMotor.hMotorRightKnee = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_MOTOR, 1.1457f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
  hExoskeletonFoshan4DOFTMotor.hMotorRightHip = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_MOTOR, 1.2339f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
	hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_KNEE_MOTOR, 1.1457f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
  hExoskeletonFoshan4DOFTMotor.hMotorLeftHip = AK10_9_Create(&hcan2, CAN_ID_TMOTOR_EXOSKELETON_RIGHT_HIP_MOTOR, 1.2339f, 1.0f, 0.0f, \
                                        14.043f, hExoskeletonFoshan4DOFTMotor.controlLoopPeriod * 4.0f, -1.9645f, 0.9651f, 0.0001551f, 0.0003103f, 0.0001551f);
	
  
  hExoskeletonFoshan4DOFTMotor.motorCmdSequence = FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED;
}

void Foshan4DOFExoskeletonTMotor_CenterControl(void)
{
	switch (hExoskeletonFoshan4DOFTMotor.task)
  {
  	case FOSHAN_4DOF_EXO_TASK_IDLE:
    {
  		break;
    }
  	case FOSHAN_4DOF_EXO_TASK_MANUAL_CONTROL:
    {
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_GRAVITY_COMPENSATION:
    {
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_ENABLE_MOTORS:
    {
      if (hExoskeletonFoshan4DOFTMotor.motorCmdSequence == FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP)
        AK10_9_MITMode_EnableMotor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip);
      hExoskeletonFoshan4DOFTMotor.motorCmdSequence += 1;
  		break;
    }
    case FOSHAN_4DOF_EXO_TASK_DISABLE_MOTORS:
    {
  		break;
    }
  	default:
  		break;
  }
	
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftHip, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorLeftKnee, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorRightHip, 300);
	AK10_9_MotorStatusMonitor(&hExoskeletonFoshan4DOFTMotor.hMotorRightKnee, 300);
}
