/**
 * @file uart_protocol_test.c
 * @brief Host/cross-compiler용 순수 UART codec golden test
 */

#include "uart_protocol.h"

#include <stdint.h>
#include <string.h>

#define CHECK(condition) \
  do {                   \
    if (!(condition)) {  \
      return __LINE__;   \
    }                    \
  } while (0)

int main(void) {
  static const uint8_t expected[] = {
      0xAAU, 0x04U, 0x01U, 0xF4U, 0x01U, 0xF4U, 0x01U, 0xB1U, 0x55U};
  static const uint8_t payload[] = {0xF4U, 0x01U, 0xF4U, 0x01U};
  uint8_t encoded[UART_PROTOCOL_MAX_FRAME];
  uint8_t encoded_length = 0U;
  UartParser_t parser;
  UartFrame_t frame;
  UartParseResult_t result = UART_PARSE_INCOMPLETE;

  CHECK(UartProtocol_EncodeFrame(UART_CMD_SET_SPEED, payload, sizeof(payload),
                                 encoded, sizeof(encoded),
                                 &encoded_length));
  CHECK(encoded_length == sizeof(expected));
  CHECK(memcmp(encoded, expected, sizeof(expected)) == 0);

  UartProtocol_ParserInit(&parser);
  for (uint8_t index = 0U; index < encoded_length; index++) {
    result = UartProtocol_ParseByte(&parser, encoded[index], &frame);
  }
  CHECK(result == UART_PARSE_FRAME_READY);
  CHECK(frame.command == UART_CMD_SET_SPEED);
  CHECK(frame.length == sizeof(payload));
  CHECK(memcmp(frame.payload, payload, sizeof(payload)) == 0);
  return 0;
}
