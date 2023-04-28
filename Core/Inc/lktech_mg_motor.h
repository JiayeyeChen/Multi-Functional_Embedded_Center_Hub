#ifndef LKTECH_MG_MOTOR_H
#define LKTECH_MG_MOTOR_H

#include "system_periphrals.h"
#include "can_bus.h"

#define             LKTECH_MG_COMMAND_READ_PID                    0x30
#define             LKTECH_MG_COMMAND_SET_PID_TO_RAM              0x31
#define             LKTECH_MG_COMMAND_SET_PID_TO_ROM              0x32
#define             LKTECH_MG_COMMAND_READ_ACCELERATION           0x33
#define             LKTECH_MG_COMMAND_SET_ACCELERATION_TO_RAM     0x34
#define             LKTECH_MG_COMMAND_READ_ENCODER                0x90
#define             LKTECH_MG_COMMAND_ZEROING_BY_VALUE            0x91
#define             LKTECH_MG_COMMAND_ZEROING_BY_CURRENT_POSITION 0x19
#define             LKTECH_MG_COMMAND_READ_ANGLE_MULTI_TURN       0x92
#define             LKTECH_MG_COMMAND_READ_ANGLE_SINGLE_TURN      0x94
#define             LKTECH_MG_COMMAND_READ_CONDITION1_N_ERROR     0x9A
#define             LKTECH_MG_COMMAND_CLEAR_ERROR                 0x9B
#define             LKTECH_MG_COMMAND_READ_CONDITION2             0x9C
#define             LKTECH_MG_COMMAND_READ_CONDITION3             0x9D
#define             LKTECH_MG_COMMAND_SHUT_DOWN                   0x80
#define             LKTECH_MG_COMMAND_DISABLE                     0x81
#define             LKTECH_MG_COMMAND_ENABLE                      0x88
#define             LKTECH_MG_COMMAND_OPEN_LOOP_CONTROL           0xA0
#define             LKTECH_MG_COMMAND_CURRENT_CONTROL             0xA1
#define             LKTECH_MG_COMMAND_VELOCITY_CONTROL            0xA2
#define             LKTECH_MG_COMMAND_POSITION_CONTROL1           0xA3
#define             LKTECH_MG_COMMAND_POSITION_CONTROL2           0xA4
#define             LKTECH_MG_COMMAND_POSITION_CONTROL3           0xA5
#define             LKTECH_MG_COMMAND_POSITION_CONTROL4           0xA6
#define             LKTECH_MG_COMMAND_POSITION_CONTROL5           0xA7
#define             LKTECH_MG_COMMAND_POSITION_CONTROL6           0xA8


enum LKTECH_MG_CAN_BUS_TASK
{
  LKTECH_MG_CAN_BUS_TASK_NONE,
  LKTECH_MG_CAN_BUS_TASK_READ_PID,
  LKTECH_MG_CAN_BUS_TASK_SET_PID_TO_RAM,
  LKTECH_MG_CAN_BUS_TASK_READ_ACCELERATION,
  LETECH_MG_CAN_BUS_TASK_SET_ACCELERATION_TO_RAM,
  LETECH_MG_CAN_BUS_TASK_ZEROING_BY_CURRENT_POSITION,
  LETECH_MG_CAN_BUS_TASK_READ_ANGLE_SINGLE_TURN,
  LETECH_MG_CAN_BUS_TASK_READ_ANGLE_MULTI_TURN,
  LETECH_MG_CAN_BUS_TASK_READ_CONDITION1_AND_ERROR,
  LETECH_MG_CAN_BUS_TASK_READ_CONDITION2,
  LETECH_MG_CAN_BUS_TASK_READ_CONDITION3,
  LETECH_MG_CAN_BUS_TASK_SHUTDOWN,
  LETECH_MG_CAN_BUS_TASK_DISABLE,
  LETECH_MG_CAN_BUS_TASK_ENABLE,
  LETECH_MG_CAN_BUS_TASK_CURRENT_CONTROL,
  LETECH_MG_CAN_BUS_TASK_SPEED_CONTROL,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_1_MULTI_TURN,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_2_MULTI_TURN,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_3_SINGLE_TURN,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_4_SINGLE_TURN,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_5_INCREMENT,
  LETECH_MG_CAN_BUS_TASK_POSITION_CONTROL_6_INCREMENT
};

typedef struct
{
  CAN_HandleTypeDef*           hcan;
  uint8_t                      motorID;
  uint32_t                     canID;
  enum LKTECH_MG_CAN_BUS_TASK  task;
	float												 gearRatio;
  //CAN BUS transmit
  uint8_t                      txBuf[8];
  uint32_t*                    pTxMailbox;
  CAN_TxHeaderTypeDef          txHeader;
  //PID
  float                        positionLoopKp;
  float                        positionLoopKi;
  float                        speedLoopKp;
  float                        speedLoopKi;
  float                        currentLoopKp;
  float                        currentLoopKi;
  //Motor Control
  float                        kt;
  union Int32UInt8             accelerationDeg;
  union UInt32UInt8            angleRaw;
  union FloatUInt8             angle;
  union Int64UInt8             angleMultiTurnRaw;
  union FloatUInt8             angleMultiTurn;
  union Int16UInt8             currentRaw;
  union FloatUInt8             current;
  union FloatUInt8             torque;
  union Int16UInt8             speedRawDeg;
  union FloatUInt8             speedDeg;
  union UInt16UInt8            encoderRaw;
  union FloatUInt8             encoder;
  union FloatUInt8             temperature;
  union FloatUInt8             voltage;
  uint8_t                      errorCode;
  union FloatUInt8             positionControlSingleTurnSet;
  union FloatUInt8             positionControlMultiTurnSet;
	union FloatUInt8             positionControlIncrementSet;
  union FloatUInt8             velocityControlSet;
  union FloatUInt8             currentControlSet;
}LKTECH_MG_Handle;

void LKTECH_MG_Init(LKTECH_MG_Handle* hmotor, CAN_HandleTypeDef* hcan, uint32_t motor_id, float gear_ratio, float kt);
void LKTECH_MG_SendSingleCommand(LKTECH_MG_Handle* hmotor, uint8_t command, uint8_t task_name);
void LKTECH_MG_ReadPID(LKTECH_MG_Handle* hmotor);
void LKTECH_MG_GetFeedback(LKTECH_MG_Handle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[]);
void LKTECH_MG_SetPIDtoRAM(LKTECH_MG_Handle* hmotor, uint8_t position_kp, uint8_t position_ki, \
                                                     uint8_t velocity_kp, uint8_t velocity_ki, \
                                                     uint8_t current_kp, uint8_t current_ki);
void LKTECH_MG_ReadAcceleration(LKTECH_MG_Handle* hmotor);
void LETECH_MG_SetAccelerationToRAM(LKTECH_MG_Handle* hmotor, int32_t acceleration_deg);
void LETECH_MG_ZeroingByCurrentPosition(LKTECH_MG_Handle* hmotor);
void LETECH_MG_ReadAngleSingleTurn(LKTECH_MG_Handle* hmotor);
void LETECH_MG_ReadAngleMultiTurn(LKTECH_MG_Handle* hmotor);
void LETECH_MG_ReadCondition1andError(LKTECH_MG_Handle* hmotor);
void LETECH_MG_ReadCondition2(LKTECH_MG_Handle* hmotor);
void LETECH_MG_ReadCondition3(LKTECH_MG_Handle* hmotor);
void LETECH_MG_Shutdown(LKTECH_MG_Handle* hmotor);
void LETECH_MG_Disable(LKTECH_MG_Handle* hmotor);
void LETECH_MG_Enable(LKTECH_MG_Handle* hmotor);
void LETECH_MG_CurrentControl(LKTECH_MG_Handle* hmotor, float iq);
void LETECH_MG_SpeedControl(LKTECH_MG_Handle* hmotor, float spd);
void LETECH_MG_PositionControl1MultiTurn(LKTECH_MG_Handle* hmotor, float pos_multi);
void LETECH_MG_PositionControl2MultiTurn(LKTECH_MG_Handle* hmotor, float pos_multi, float vel_limit);
void LETECH_MG_PositionControl5Increment(LKTECH_MG_Handle* hmotor, float pos_incre);
void LETECH_MG_PositionControl6Increment(LKTECH_MG_Handle* hmotor, float pos_incre, float vel_limit);
//void LETECH_MG_MultiMotorsCurrentControl();
#endif
