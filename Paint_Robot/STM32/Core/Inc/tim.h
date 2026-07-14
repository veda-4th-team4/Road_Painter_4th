/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.h
 * @brief   TIM1 servo PWM and TIM2 20 kHz motor tick declarations
 * @details TIM1_CH1은 PA8에서 50 Hz 서보 PWM을 출력하고, TIM2 update
 *          interrupt는 모터 실시간 제어 tick을 제공합니다.
 ******************************************************************************
 */
/* USER CODE END Header */

#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern TIM_HandleTypeDef htim1; /**< 노즐 서보 50 Hz PWM 타이머 핸들. */
extern TIM_HandleTypeDef htim2; /**< 모터 제어 20 kHz tick 타이머 핸들. */

/**
 * @brief TIM1_CH1을 1 MHz counter, 20 ms period PWM으로 초기화합니다.
 */
void MX_TIM1_Init(void);

/**
 * @brief TIM2를 ROBOT_MOTOR_TICK_HZ update timer로 초기화합니다.
 */
void MX_TIM2_Init(void);

/**
 * @brief TIM1 초기화 후 PA8을 TIM1_CH1 alternate function으로 설정합니다.
 * @param htim post-initialization 대상 타이머 핸들입니다.
 */
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */
