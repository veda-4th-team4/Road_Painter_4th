/**
 ******************************************************************************
 * @file    app_rtos.h
 * @brief   Paint Robot FreeRTOS application composition
 ******************************************************************************
 */

#ifndef __APP_RTOS_H__
#define __APP_RTOS_H__

#include "main.h"

/** @brief debugger/진단용 task stack 최소 여유량 [word]입니다. */
typedef struct {
  uint32_t communication_free_words;
  uint32_t control_free_words;
} AppRtosStackWatermark_t;

HAL_StatusTypeDef AppRtos_Init(UART_HandleTypeDef *command_uart);
void AppRtos_GetStackWatermarks(AppRtosStackWatermark_t *watermarks);

#endif /* __APP_RTOS_H__ */
