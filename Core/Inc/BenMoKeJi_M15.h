#ifndef BENMOKEJI_M15_H
#define BENMOKEJI_M15_H

#include "system_periphrals.h"
#include "can_bus.h"

#define				BENMOKEJI_M15_MODE_VOLTAGE_CONTROL			0x00
#define				BENMOKEJI_M15_MODE_CURRENT_CONTROL			0x01
#define				BENMOKEJI_M15_MODE_SPEED_CONTROL				0x02
#define				BENMOKEJI_M15_MODE_POSITION_CONTROL			0x03
#define				BENMOKEJI_M15_MODE_DISABLED							0x09
#define				BENMOKEJI_M15_MODE_ENABLED							0x0A

#define				BENMOKEJI_M15_FEEDBACK_MODE_AUTO				0x00
#define				BENMOKEJI_M15_FEEDBACK_MODE_MANUAL			0x01

enum BENMOKEJI_MODE
{
	BENMOKEJI_MODE_DISABLED,
	BENMOKEJI_MODE_ENABLED,
	BENMOKEJI_MODE_POSITION,
	BENMOKEJI_MODE_VELOCITY,
	BENMOKEJI_MODE_CURRENT
};

typedef struct
{
	CAN_HandleTypeDef*    hcan;
	uint8_t								motorID;
	uint8_t								canIDFeedback;
	uint8_t								canIDCommand;
	enum BENMOKEJI_MODE   mode;
  uint8_t               errorCode;
  //CAN BUS transmit
  uint8_t               txBuf[8];
  uint32_t*             pTxMailbox;
  CAN_TxHeaderTypeDef   txHeader;
	
	union Int16UInt8			speedRaw, currentRaw, positionRaw;
	union FloatUInt8			speedRealDeg, currentReal, positionRealDeg;
	union FloatUInt8			speedSetDeg, currentSet, positionSetDeg;
}BENMOKEJI_M15_Handle;

void BENMOKEJI_M15_Init(BENMOKEJI_M15_Handle* hmotor, CAN_HandleTypeDef* hcan, uint8_t motor_id);
void BENMOKEJI_M15_GetFeedback(BENMOKEJI_M15_Handle* hmotor, CAN_RxHeaderTypeDef* rxheader, uint8_t rxbuf[]);
void BENMOKEJI_M15_SetFeedbackMode(BENMOKEJI_M15_Handle* hmotor, uint8_t mode, uint8_t auto_feedback_duration);
void BENMOKEJI_M15_SendControlValueSingleMotor(BENMOKEJI_M15_Handle* hmotor, int16_t val);
void BENMODEJI_M15_PositionControlSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val);
void BENMODEJI_M15_VelocityControlDegSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val);
void BENMODEJI_M15_CurrentControlSingleMotor(BENMOKEJI_M15_Handle* hmotor, float val);
void BENMOKEJI_M15_SetMode(BENMOKEJI_M15_Handle* hmotor, uint8_t mode);
void BENMOKEJI_M15_SetCANID(BENMOKEJI_M15_Handle* hmotor, uint8_t new_id);
void BENMOKEJI_M15_SetManualFeedbackMode(void);
void BENMOKEJI_M15_RequestForManualFeedback_VelCurTem(BENMOKEJI_M15_Handle* hmotor);

#endif
