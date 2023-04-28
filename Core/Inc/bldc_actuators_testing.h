#ifndef BLDC_ACTUATORS_TESTING_H
#define BLDC_ACTUATORS_TESTING_H

#include "lktech_mg_motor.h"
#include "BenMoKeJi_M15.h"
#include "usb.h"

void LKTECH_Testing_DataSlotUpdate(void);
void LKTECH_MotorTest_Set_Datalog_Label(void);

extern LKTECH_MG_Handle     hLKTECH;
extern BENMOKEJI_M15_Handle hBENMOKEJI;
#endif
