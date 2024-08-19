#include "opticalencoder_acceleration_test.h"




union FloatUInt8 datalog_slot_cui[3];
CUIAMT222BHandle hCUIEncoder;
SerialProtocolHandle hSerial;
float pre_angle, cur_angle, pre_speed, cur_speed, pre_acc, cur_acc;
LowPassFilterHandle hFilterSpeed, hFilterAcc;

void OPTICALENCODERTEST_SetDatalogLabel(void)
{
	SERIALPROTOCOL_SendDataSlotLabel(&hSerial, "3", "Angle", "Speed", "Acceleration");
}
