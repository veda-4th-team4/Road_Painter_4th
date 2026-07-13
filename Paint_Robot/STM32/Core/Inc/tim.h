/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   TIM1 servo PWM and TIM2 20 kHz motor tick declarations
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

void MX_TIM1_Init(void);
void MX_TIM2_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
