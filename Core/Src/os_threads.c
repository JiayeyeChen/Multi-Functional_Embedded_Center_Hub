#include "os_threads.h"

osThreadId_t AK10_CalibrationnTaskHandle;
const osThreadAttr_t AK10_Calibration_attributes = {
  .name = "AK10_Calibration",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};

osThreadId_t LCDTaskHandle;
const osThreadAttr_t LCDTask_attributes = {
  .name = "LCD",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};


void OSThreads_Init(void)
{
  AK10_CalibrationnTaskHandle = osThreadNew(AK10Calibration_Task, NULL, &AK10_Calibration_attributes);
  LCDTaskHandle = osThreadNew(LCD_Task, NULL, &LCDTask_attributes);
}
