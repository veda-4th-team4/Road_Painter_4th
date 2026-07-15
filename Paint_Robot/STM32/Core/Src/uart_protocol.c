/**
 ******************************************************************************
 * @file    uart_protocol.c
 * @brief   HAL/RTOS 비종속 UART frame codec 구현
 ******************************************************************************
 */

#include "uart_protocol.h"

#include <stddef.h>
#include <string.h>

/** @brief CRC-8(poly 0x07)에 한 바이트를 누적합니다. */
static uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0U; bit < 8U; bit++) {
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U)
                        : (uint8_t)(crc << 1U);
  }
  return crc;
}

/** @brief parser를 STX 대기 상태로 되돌립니다. */
static void parser_reset(UartParser_t *parser) {
  parser->state = UART_PARSER_WAIT_STX;
  parser->frame.command = 0U;
  parser->frame.length = 0U;
  parser->payload_index = 0U;
  parser->calculated_crc = 0U;
  parser->received_crc = 0U;
}

void UartProtocol_ParserInit(UartParser_t *parser) {
  if (parser != NULL) {
    memset(parser, 0, sizeof(*parser));
    parser_reset(parser);
  }
}

UartParseResult_t UartProtocol_ParseByte(UartParser_t *parser, uint8_t byte,
                                         UartFrame_t *completed_frame) {
  if (parser == NULL || completed_frame == NULL) {
    return UART_PARSE_ERROR;
  }

  switch (parser->state) {
  case UART_PARSER_WAIT_STX:
    if (byte == UART_FRAME_STX) {
      parser->state = UART_PARSER_LENGTH;
    }
    break;

  case UART_PARSER_LENGTH:
    if (byte > UART_PROTOCOL_MAX_PAYLOAD) {
      parser_reset(parser);
      return UART_PARSE_ERROR;
    }
    parser->frame.length = byte;
    parser->payload_index = 0U;
    parser->calculated_crc = crc8_update(0U, byte);
    parser->state = UART_PARSER_COMMAND;
    break;

  case UART_PARSER_COMMAND:
    parser->frame.command = byte;
    parser->calculated_crc = crc8_update(parser->calculated_crc, byte);
    parser->state = (parser->frame.length == 0U) ? UART_PARSER_CRC
                                                 : UART_PARSER_PAYLOAD;
    break;

  case UART_PARSER_PAYLOAD:
    parser->frame.payload[parser->payload_index++] = byte;
    parser->calculated_crc = crc8_update(parser->calculated_crc, byte);
    if (parser->payload_index >= parser->frame.length) {
      parser->state = UART_PARSER_CRC;
    }
    break;

  case UART_PARSER_CRC:
    parser->received_crc = byte;
    parser->state = UART_PARSER_ETX;
    break;

  case UART_PARSER_ETX:
    if (byte == UART_FRAME_ETX &&
        parser->received_crc == parser->calculated_crc) {
      *completed_frame = parser->frame;
      parser_reset(parser);
      return UART_PARSE_FRAME_READY;
    }

    parser_reset(parser);
    if (byte == UART_FRAME_STX) {
      parser->state = UART_PARSER_LENGTH;
    }
    return UART_PARSE_ERROR;

  default:
    parser_reset(parser);
    return UART_PARSE_ERROR;
  }

  return UART_PARSE_INCOMPLETE;
}

uint8_t UartProtocol_EncodeFrame(uint8_t command, const uint8_t *payload,
                                 uint8_t payload_length, uint8_t *output,
                                 uint8_t output_capacity,
                                 uint8_t *output_length) {
  uint8_t frame_length =
      (uint8_t)(payload_length + UART_PROTOCOL_FRAME_OVERHEAD);

  if (output == NULL || output_length == NULL ||
      payload_length > UART_PROTOCOL_MAX_PAYLOAD ||
      frame_length > output_capacity ||
      (payload_length > 0U && payload == NULL)) {
    return 0U;
  }

  output[0] = UART_FRAME_STX;
  output[1] = payload_length;
  output[2] = command;
  if (payload_length > 0U) {
    memcpy(&output[3], payload, payload_length);
  }
  output[3U + payload_length] =
      UartProtocol_Crc8(&output[1], (uint16_t)payload_length + 2U);
  output[4U + payload_length] = UART_FRAME_ETX;
  *output_length = frame_length;
  return 1U;
}

uint8_t UartProtocol_Crc8(const uint8_t *data, uint16_t length) {
  uint8_t crc = 0U;
  if (data == NULL) {
    return crc;
  }

  for (uint16_t index = 0U; index < length; index++) {
    crc = crc8_update(crc, data[index]);
  }
  return crc;
}

int16_t UartProtocol_ReadI16Le(const uint8_t *data) {
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8U));
}

uint16_t UartProtocol_ReadU16Le(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

void UartProtocol_WriteU32Le(uint8_t *data, uint32_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8U);
  data[2] = (uint8_t)(value >> 16U);
  data[3] = (uint8_t)(value >> 24U);
}
