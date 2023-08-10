#include "bldc_actuators_testing.h"

LKTECH_MG_Handle     hLKTECH;


void LKTECH_Testing_DataSlotUpdate(void)
{
  uint8_t ptr = 0;
  dataSlots_LKTECH_MG_MotorTest[ptr++].f = hLKTECH.angle.f;
  dataSlots_LKTECH_MG_MotorTest[ptr++].f = hLKTECH.speedDeg.f;
  dataSlots_LKTECH_MG_MotorTest[ptr++].f = hLKTECH.torque.f;
  dataSlots_LKTECH_MG_MotorTest[ptr++].f = hLKTECH.temperature.f;
}

void LKTECH_MotorTest_Set_Datalog_Label(void)
{
  USB_SendDataSlotLabel("4", "Pos", "Vel", "Torque", "Temp");
}
