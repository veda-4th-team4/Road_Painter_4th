/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */
/*
 * [DRV8825 GPIO 설정 - Robot_Painter.ioc와 반드시 일치]
 * PB0 LEFT_STEP, PB1 LEFT_DIR, PB6 LEFT_EN
 * PB2 RIGHT_STEP, PB5 RIGHT_DIR, PB7 RIGHT_EN
 * 모두 Output Push-Pull / No Pull / High Speed.
 * nENBL은 Active-Low이므로 LEFT_EN/RIGHT_EN 초기 출력은 HIGH(비활성).
 *
 * 이 주석은 CubeMX 코드 재생성 후에도 USER CODE 영역에 보존됩니다.
 */
/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*
   * 모터 출력의 안전한 부팅 상태:
   * - STEP/DIR = LOW
   * - EN1/EN2  = HIGH (DRV8825 nENBL 비활성)
   * Motor_Init() 이후 실제 명령을 실행할 때만 EN을 LOW로 내립니다.
   */
  HAL_GPIO_WritePin(GPIOB,
                    LEFT_STEP_Pin | LEFT_DIR_Pin | RIGHT_STEP_Pin |
                        RIGHT_DIR_Pin,
                    GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, LEFT_EN_Pin | RIGHT_EN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*
   * DRV8825 제어 GPIO:
   * PB0=STEP1, PB1=DIR1, PB6=EN1, PB2=STEP2, PB5=DIR2, PB7=EN2
   * Push-Pull / No Pull / High Speed 출력으로 설정합니다.
   */
  GPIO_InitStruct.Pin = LEFT_STEP_Pin | LEFT_DIR_Pin | LEFT_EN_Pin |
                        RIGHT_STEP_Pin | RIGHT_DIR_Pin | RIGHT_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
