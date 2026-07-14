/**
 ******************************************************************************
 * @file    uart_transport.c
 * @brief   USART1 interrupt I/O와 정적 FreeRTOS buffer 구현
 ******************************************************************************
 */

#include "uart_transport.h"

#include "stream_buffer.h"
#include "robot_config.h"
#include "task.h"
#include "uart_protocol.h"

#include <string.h>

typedef struct {
  uint8_t bytes[UART_PROTOCOL_MAX_FRAME];
  uint8_t length;
} UartTxFrame_t;

static UART_HandleTypeDef *command_uart;
static uint8_t rx_byte;

static StaticStreamBuffer_t rx_stream_control;
static uint8_t rx_stream_storage[ROBOT_UART_RX_RING_SIZE + 1U];
static StreamBufferHandle_t rx_stream;

static UartTxFrame_t tx_queue[ROBOT_UART_TX_QUEUE_DEPTH];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_busy;
static volatile uint8_t error_flags;

/** @brief 현재 TX tail frame의 interrupt 송신을 시작합니다. */
static HAL_StatusTypeDef start_current_tx(void) {
  HAL_StatusTypeDef status;

  if (command_uart == NULL || !tx_busy || tx_tail == tx_head) {
    return HAL_OK;
  }

  status = HAL_UART_Transmit_IT(command_uart, tx_queue[tx_tail].bytes,
                                tx_queue[tx_tail].length);
  if (status != HAL_OK) {
    tx_busy = 0U;
    error_flags |= UART_TRANSPORT_ERROR_TX_OVERFLOW;
  }
  return status;
}

HAL_StatusTypeDef UartTransport_Init(UART_HandleTypeDef *uart) {
  if (uart == NULL) {
    return HAL_ERROR;
  }

  command_uart = uart;
  tx_head = 0U;
  tx_tail = 0U;
  tx_busy = 0U;
  error_flags = 0U;
  rx_stream = xStreamBufferCreateStatic(
      sizeof(rx_stream_storage), 1U, rx_stream_storage, &rx_stream_control);
  return (rx_stream != NULL) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef UartTransport_Start(void) {
  if (command_uart == NULL || rx_stream == NULL) {
    return HAL_ERROR;
  }
  return HAL_UART_Receive_IT(command_uart, &rx_byte, 1U);
}

size_t UartTransport_Read(uint8_t *buffer, size_t capacity,
                          TickType_t timeout_ticks) {
  if (rx_stream == NULL || buffer == NULL || capacity == 0U) {
    return 0U;
  }
  return xStreamBufferReceive(rx_stream, buffer, capacity, timeout_ticks);
}

uint8_t UartTransport_Send(const uint8_t *frame, uint8_t length) {
  uint8_t next;
  uint8_t should_start = 0U;

  if (frame == NULL || length == 0U ||
      length > UART_PROTOCOL_MAX_FRAME) {
    return 0U;
  }

  taskENTER_CRITICAL();
  next = (uint8_t)((tx_head + 1U) % ROBOT_UART_TX_QUEUE_DEPTH);
  if (next == tx_tail) {
    error_flags |= UART_TRANSPORT_ERROR_TX_OVERFLOW;
    taskEXIT_CRITICAL();
    return 0U;
  }

  memcpy(tx_queue[tx_head].bytes, frame, length);
  tx_queue[tx_head].length = length;
  tx_head = next;
  if (!tx_busy) {
    tx_busy = 1U;
    should_start = 1U;
  }
  taskEXIT_CRITICAL();

  if (should_start) {
    (void)start_current_tx();
  }
  return 1U;
}

uint8_t UartTransport_TakeErrorFlags(void) {
  uint8_t flags;
  taskENTER_CRITICAL();
  flags = error_flags;
  error_flags = 0U;
  taskEXIT_CRITICAL();
  return flags;
}

void UartTransport_DiscardRx(void) {
  if (rx_stream == NULL) {
    return;
  }

  /* USART1 priority 6은 BASEPRI=5 critical section 동안 실행되지 않습니다. */
  taskENTER_CRITICAL();
  (void)xStreamBufferReset(rx_stream);
  taskEXIT_CRITICAL();
}

void UartTransport_RxCpltCallback(UART_HandleTypeDef *uart) {
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (command_uart == NULL || rx_stream == NULL ||
      uart->Instance != command_uart->Instance) {
    return;
  }

  if (xStreamBufferSendFromISR(rx_stream, &rx_byte, 1U,
                               &higher_priority_task_woken) != 1U) {
    error_flags |= UART_TRANSPORT_ERROR_RX;
  }
  if (HAL_UART_Receive_IT(command_uart, &rx_byte, 1U) != HAL_OK) {
    error_flags |= UART_TRANSPORT_ERROR_RX;
  }
  portYIELD_FROM_ISR(higher_priority_task_woken);
}

void UartTransport_TxCpltCallback(UART_HandleTypeDef *uart) {
  if (command_uart == NULL || uart->Instance != command_uart->Instance) {
    return;
  }

  tx_tail = (uint8_t)((tx_tail + 1U) % ROBOT_UART_TX_QUEUE_DEPTH);
  if (tx_tail != tx_head) {
    tx_busy = 1U;
    (void)start_current_tx();
  } else {
    tx_busy = 0U;
  }
}

void UartTransport_ErrorCallback(UART_HandleTypeDef *uart) {
  if (command_uart == NULL || uart->Instance != command_uart->Instance) {
    return;
  }

  error_flags |= UART_TRANSPORT_ERROR_RX;
  __HAL_UART_CLEAR_OREFLAG(uart);
  if (uart->RxState == HAL_UART_STATE_READY) {
    (void)HAL_UART_Receive_IT(command_uart, &rx_byte, 1U);
  }
}
