/**
 ******************************************************************************
 * @file    uart_protocol.h
 * @brief   V-[HW] UART 통신 PDF 기반 RPi <-> STM32 바이너리 프로토콜
 ******************************************************************************
 */

#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#include "robot_config.h"

#define UART_FRAME_STX 0xAAU
#define UART_FRAME_ETX 0x55U

typedef enum {
  UART_CMD_SET_SPEED   = 0x01U,
  UART_CMD_NOZZLE      = 0x02U,
  UART_CMD_ESTOP       = 0x03U,
  UART_CMD_CLEAR_ESTOP = 0x04U, /* SRS 안전 보완 확장 */
  UART_CMD_STATUS      = 0x81U
} UartCommand_t;

/* 0x81 STATUS flags */
#define STATUS_FLAG_MOVING      (1U << 0)
#define STATUS_FLAG_ESTOP       (1U << 1)
#define STATUS_FLAG_TIMEOUT     (1U << 2)
#define STATUS_FLAG_NOZZLE      (1U << 3)
#define STATUS_FLAG_RX_ERROR    (1U << 4)
#define STATUS_FLAG_TX_OVERFLOW (1U << 5)

/* 0x03 ESTOP reason 및 내부 안전 원인 */
typedef enum {
  ESTOP_REASON_NONE         = 0x00U,
  ESTOP_REASON_RPI          = 0x01U,
  ESTOP_REASON_UART_TIMEOUT = 0x02U,
  ESTOP_REASON_UART_ERROR   = 0x03U,
  ESTOP_REASON_INTERNAL     = 0x7FU
} EStopReason_t;

/**
 * @brief 프로토콜 상태와 USART1 1-byte interrupt RX를 초기화합니다.
 */
HAL_StatusTypeDef UartProtocol_Init(UART_HandleTypeDef *command_uart,
                                    UART_HandleTypeDef *debug_uart);

/**
 * @brief 메인 루프에서 반복 호출합니다. RX 파싱, watchdog, STATUS를 처리합니다.
 */
void UartProtocol_Process(uint32_t now_ms);

/* 아래 콜백은 main.c의 HAL 콜백에서 전달합니다. */
void UartProtocol_RxCpltCallback(UART_HandleTypeDef *huart);
void UartProtocol_TxCpltCallback(UART_HandleTypeDef *huart);
void UartProtocol_ErrorCallback(UART_HandleTypeDef *huart);

/* 테스트/상위 모듈에서 동일 CRC를 사용할 수 있도록 공개합니다. */
uint8_t UartProtocol_Crc8(const uint8_t *data, uint16_t len);

#endif /* __UART_PROTOCOL_H__ */
