/**
 ******************************************************************************
 * @file    uart_transport.h
 * @brief   FreeRTOS 기반 USART1 byte transport
 ******************************************************************************
 */

#ifndef __UART_TRANSPORT_H__
#define __UART_TRANSPORT_H__

#include "FreeRTOS.h"
#include "main.h"

#include <stddef.h>
#include <stdint.h>

#define UART_TRANSPORT_ERROR_RX          (1U << 0)
#define UART_TRANSPORT_ERROR_TX_OVERFLOW (1U << 1)

HAL_StatusTypeDef UartTransport_Init(UART_HandleTypeDef *uart);
HAL_StatusTypeDef UartTransport_Start(void);
size_t UartTransport_Read(uint8_t *buffer, size_t capacity,
                          TickType_t timeout_ticks);
uint8_t UartTransport_Send(const uint8_t *frame, uint8_t length);
uint8_t UartTransport_TakeErrorFlags(void);
void UartTransport_DiscardRx(void);

void UartTransport_RxCpltCallback(UART_HandleTypeDef *uart);
void UartTransport_TxCpltCallback(UART_HandleTypeDef *uart);
void UartTransport_ErrorCallback(UART_HandleTypeDef *uart);

#endif /* __UART_TRANSPORT_H__ */
