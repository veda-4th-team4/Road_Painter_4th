/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define SIG_SERVO_Pin GPIO_PIN_8
#define SIG_SERVO_GPIO_Port GPIOA
#define LEFT_STEP_Pin GPIO_PIN_0
#define LEFT_STEP_GPIO_Port GPIOB
#define LEFT_DIR_Pin GPIO_PIN_1
#define LEFT_DIR_GPIO_Port GPIOB
#define RIGHT_STEP_Pin GPIO_PIN_2
#define RIGHT_STEP_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define RIGHT_DIR_Pin GPIO_PIN_5
#define RIGHT_DIR_GPIO_Port GPIOB
#define LEFT_EN_Pin GPIO_PIN_6
#define LEFT_EN_GPIO_Port GPIOB
#define RIGHT_EN_Pin GPIO_PIN_7
#define RIGHT_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
/**
 * @brief 모터 및 서보 핀 정의의 원본은 Robot_Painter.ioc입니다.
 * @warning 핀을 바꿀 때 main.h만 직접 수정하지 말고 CubeMX Pinout을 함께
 *          변경해야 코드 재생성 후에도 일관성이 유지됩니다.
 */

/** @brief IR 수신기 입력. TIM3_CH1 / PB4 (기존 모터 핀과 비충돌). */
#define IR_RX_Pin        GPIO_PIN_4
#define IR_RX_GPIO_Port  GPIOB
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
