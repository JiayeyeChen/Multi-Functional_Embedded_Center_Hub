#include "os_threads.h"

osThreadId_t MainTaskHandle;
const osThreadAttr_t Main_attributes = {
  .name = "Main",
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

osThreadId_t MotorTestingHandle;
const osThreadAttr_t MotorTestingTask_attributes = {
  .name = "Motor Testing",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityRealtime,
};

void OSThreads_Init(void)
{
  MainTaskHandle = osThreadNew(Main_Task, NULL, &Main_attributes);
  UITaskHandle = osThreadNew(UI_Task, NULL, &UITask_attributes);
  ADCTaskHandle = osThreadNew(ADC_Task, NULL, &ADCTask_attributes);
	MotorTestingHandle = osThreadNew(MotorTesting_Task, NULL, &MotorTestingTask_attributes);
}
