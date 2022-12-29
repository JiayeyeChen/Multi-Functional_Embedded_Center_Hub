#ifndef OS_THREADS_H
#define OS_THREADS_H

#include "cmsis_os.h"

void AK10Calibration_Task(void *argument);
void UI_Task(void *argument);
void ADC_Task(void *argument);
void UpperLimb_Task(void *argument);
void OSThreads_Init(void);









#endif
