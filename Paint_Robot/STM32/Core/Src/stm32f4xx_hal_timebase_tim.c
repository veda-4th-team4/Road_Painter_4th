/**
 ******************************************************************************
 * @file    stm32f4xx_hal_timebase_tim.c
 * @brief   FreeRTOS SysTick과 분리된 TIM5 기반 HAL 1 ms time base
 ******************************************************************************
 */

#include "stm32f4xx_hal.h"

TIM_HandleTypeDef htim5;

/**
 * @brief TIM5 update interrupt를 HAL tick source로 초기화합니다.
 * @param tick_priority TIM5 IRQ preemption priority입니다.
 * @return 초기화 및 시작 결과입니다.
 */
HAL_StatusTypeDef HAL_InitTick(uint32_t tick_priority) {
  RCC_ClkInitTypeDef clock_config;
  uint32_t flash_latency;
  uint32_t pclk1;
  uint32_t timer_clock;

  HAL_RCC_GetClockConfig(&clock_config, &flash_latency);
  pclk1 = HAL_RCC_GetPCLK1Freq();
  timer_clock = (clock_config.APB1CLKDivider == RCC_HCLK_DIV1)
                    ? pclk1
                    : (pclk1 * 2U);

  __HAL_RCC_TIM5_CLK_ENABLE();

  htim5.Instance = TIM5;
  htim5.Init.Prescaler = (timer_clock / 1000000U) - 1U;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 1000U - 1U;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim5) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM5_IRQn, tick_priority, 0U);
  HAL_NVIC_EnableIRQ(TIM5_IRQn);
  return HAL_TIM_Base_Start_IT(&htim5);
}

/** @brief HAL tick 증가를 일시 중지합니다. */
void HAL_SuspendTick(void) {
  __HAL_TIM_DISABLE_IT(&htim5, TIM_IT_UPDATE);
}

/** @brief HAL tick 증가를 재개합니다. */
void HAL_ResumeTick(void) {
  __HAL_TIM_ENABLE_IT(&htim5, TIM_IT_UPDATE);
}

/**
 * @brief TIM5 update 완료 시 HAL millisecond tick을 증가시킵니다.
 * @param htim callback을 발생시킨 timer handle입니다.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
}
