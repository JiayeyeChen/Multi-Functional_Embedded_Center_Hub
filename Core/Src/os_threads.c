#include "os_threads.h"

osThreadId_t AK10_CalibrationnTaskHandle;
const osThreadAttr_t AK10_Calibration_attributes = {
  .name = "AK10_Calibration",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t UITaskHandle;
const osThreadAttr_t UITask_attributes = {
  .name = "UI",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t ADCTaskHandle;
const osThreadAttr_t ADCTask_attributes = {
  .name = "UI",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityRealtime,
};


void OSThreads_Init(void)
{
  AK10_CalibrationnTaskHandle = osThreadNew(AK10Calibration_Task, NULL, &AK10_Calibration_attributes);
  UITaskHandle = osThreadNew(UI_Task, NULL, &UITask_attributes);
  ADCTaskHandle = osThreadNew(ADC_Task, NULL, &ADCTask_attributes);
}
