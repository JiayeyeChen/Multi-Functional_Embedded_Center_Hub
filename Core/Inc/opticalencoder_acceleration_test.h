#ifndef OPTICALENCODER_ACCELERATION_TEST_H
#define OPTICALENCODER_ACCELERATION_TEST_H

#include "cui_amt222b_encoder.h"
#include "serial_protocol.h"
#include "common.h"

void OPTICALENCODERTEST_SetDatalogLabel(void);




extern union FloatUInt8 datalog_slot_cui[3];
extern CUIAMT222BHandle hCUIEncoder;
extern SerialProtocolHandle hSerial;
extern float pre_angle, cur_angle, pre_speed, cur_speed, pre_acc, cur_acc;
extern LowPassFilterHandle hFilterSpeed, hFilterAcc;










#endif
