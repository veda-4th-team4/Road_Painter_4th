/**
 ******************************************************************************
 * @file    uart_protocol.h
 * @brief   HAL/RTOS 비종속 RPi <-> STM32 UART wire codec
 * @details 기존 규약 AA|LEN|CMD|PAYLOAD|CRC8|55와 Little-Endian payload를
 *          바이트 단위로 고정합니다. 이 모듈은 I/O와 명령 실행을 수행하지 않습니다.
 ******************************************************************************
 */

#ifndef __UART_PROTOCOL_H__
#define __UART_PROTOCOL_H__

#include <stdint.h>

#define UART_FRAME_STX              0xAAU
#define UART_FRAME_ETX              0x55U
#define UART_PROTOCOL_MAX_PAYLOAD   16U
#define UART_PROTOCOL_FRAME_OVERHEAD 5U
#define UART_PROTOCOL_MAX_FRAME     \
  (UART_PROTOCOL_MAX_PAYLOAD + UART_PROTOCOL_FRAME_OVERHEAD)

#define UART_SET_SPEED_PAYLOAD_LEN         4U
#define UART_NOZZLE_PAYLOAD_LEN            1U
#define UART_ESTOP_PAYLOAD_LEN             1U
#define UART_CLEAR_ESTOP_PAYLOAD_LEN       2U
#define UART_SET_CONTROL_MODE_PAYLOAD_LEN  1U
#define UART_STATUS_PAYLOAD_LEN            9U

/** @brief 기존 RPi와 호환되는 command ID입니다. */
typedef enum {
  UART_CMD_SET_SPEED         = 0x01U,
  UART_CMD_NOZZLE            = 0x02U,
  UART_CMD_ESTOP             = 0x03U,
  UART_CMD_CLEAR_ESTOP       = 0x04U,
  UART_CMD_SET_CONTROL_MODE  = 0x05U,
  UART_CMD_STATUS            = 0x81U
} UartCommand_t;

#define STATUS_FLAG_MOVING      (1U << 0)
#define STATUS_FLAG_ESTOP       (1U << 1)
#define STATUS_FLAG_TIMEOUT     (1U << 2)
#define STATUS_FLAG_NOZZLE      (1U << 3)
#define STATUS_FLAG_RX_ERROR    (1U << 4)
#define STATUS_FLAG_TX_OVERFLOW (1U << 5)
/** @brief 예전 AUTO/MANUAL용. 모드 폐지 후 항상 0. */
#define STATUS_FLAG_MANUAL      (1U << 6)

/** @brief ESTOP payload 및 내부 안전정지 원인 코드입니다. */
typedef enum {
  ESTOP_REASON_NONE         = 0x00U,
  ESTOP_REASON_RPI          = 0x01U,
  ESTOP_REASON_UART_TIMEOUT = 0x02U,
  ESTOP_REASON_UART_ERROR   = 0x03U,
  ESTOP_REASON_REMOTE       = 0x04U,
  ESTOP_REASON_INTERNAL     = 0x7FU
} EStopReason_t;

/** @brief parser가 완성한 검증된 wire frame입니다. */
typedef struct {
  uint8_t command;
  uint8_t length;
  uint8_t payload[UART_PROTOCOL_MAX_PAYLOAD];
} UartFrame_t;

/** @brief incremental parser의 처리 결과입니다. */
typedef enum {
  UART_PARSE_INCOMPLETE = 0,
  UART_PARSE_FRAME_READY,
  UART_PARSE_ERROR
} UartParseResult_t;

/** @brief incremental parser 내부 단계입니다. */
typedef enum {
  UART_PARSER_WAIT_STX = 0,
  UART_PARSER_LENGTH,
  UART_PARSER_COMMAND,
  UART_PARSER_PAYLOAD,
  UART_PARSER_CRC,
  UART_PARSER_ETX
} UartParserState_t;

/** @brief task가 소유하는 UART parser 상태입니다. */
typedef struct {
  UartParserState_t state;
  UartFrame_t frame;
  uint8_t payload_index;
  uint8_t calculated_crc;
  uint8_t received_crc;
} UartParser_t;

void UartProtocol_ParserInit(UartParser_t *parser);
UartParseResult_t UartProtocol_ParseByte(UartParser_t *parser, uint8_t byte,
                                         UartFrame_t *completed_frame);
uint8_t UartProtocol_EncodeFrame(uint8_t command, const uint8_t *payload,
                                 uint8_t payload_length, uint8_t *output,
                                 uint8_t output_capacity,
                                 uint8_t *output_length);
uint8_t UartProtocol_Crc8(const uint8_t *data, uint16_t length);
int16_t UartProtocol_ReadI16Le(const uint8_t *data);
uint16_t UartProtocol_ReadU16Le(const uint8_t *data);
void UartProtocol_WriteU32Le(uint8_t *data, uint32_t value);

#endif /* __UART_PROTOCOL_H__ */
