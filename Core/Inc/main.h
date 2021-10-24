#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#define USE_STM32F4_SERIES
#include "gpio_digital_filtered_input.h"
#include "common.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
