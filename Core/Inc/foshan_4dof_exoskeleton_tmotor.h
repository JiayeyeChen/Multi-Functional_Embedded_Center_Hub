#ifndef FOSHAN_4DOF_EXOSKELETON_TMOTOR_H
#define FOSHAN_4DOF_EXOSKELETON_TMOTOR_H

#include "tmotor_ak10-9_v2.h"
#include "system_periphrals.h"

enum Foshan4DOFExoTMotorMotorSendCommandSequence
{
  FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_FINISHED,
  FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_HIP,
  FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_LEFT_KNEE,
  FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_HIP,
  FOSHAN_4DOF_EXO_TMOTOR_COMMAND_SEQUENCE_RIGHT_KNEE
};

enum Foshan4DOFExoskeletonTMotorTask
{
  FOSHAN_4DOF_EXO_TASK_OFF,
  FOSHAN_4DOF_EXO_TASK_IDLE,
  FOSHAN_4DOF_EXO_TASK_MANUAL_CONTROL,
  FOSHAN_4DOF_EXO_TASK_GRAVITY_COMPENSATION,
  FOSHAN_4DOF_EXO_TASK_ENABLE_MOTORS,
  FOSHAN_4DOF_EXO_TASK_DISABLE_MOTORS,
	FOSHAN_4DOF_EXO_TASK_ZEROING
};

typedef struct
{
	AK10_9Handle hMotorLeftHip, hMotorLeftKnee, hMotorRightHip, hMotorRightKnee;
	float controlLoopPeriod;
	enum Foshan4DOFExoTMotorMotorSendCommandSequence motorCmdSequence;
  enum Foshan4DOFExoskeletonTMotorTask task;
}Foshan4DOFExoskeletonTMotorHandle;

void Foshan4DOFExoskeletonTMotor_Init(float controlLoopPeriod);

void Foshan4DOFExoskeletonTMotor_CenterControl(void);






extern Foshan4DOFExoskeletonTMotorHandle hExoskeletonFoshan4DOFTMotor;

#endif
