#include "packet_parser.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

static UART_HandleTypeDef *target_huart;
static uint8_t rx_ring_buf[RX_RING_BUF_SIZE];
static volatile uint16_t ring_head = 0;
static volatile uint16_t ring_tail = 0;

/* Global system state variables */
volatile int16_t global_left_sps = 0;
volatile int16_t global_right_sps = 0;
volatile uint8_t global_nozzle_on = 0;
volatile uint8_t global_estop_flag = 0;

/**
 * @brief Computes CRC-8-CCITT checksum (polynomial 0x07).
 * @param data Pointer to the input data buffer.
 * @param len Length of the data in bytes.
 * @return Calculated CRC-8 value.
 */
static uint8_t get_crc8(uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x07;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

/**
 * @brief Pops a single byte from the RX ring buffer.
 * @param byte Output pointer to store the retrieved byte.
 * @return int 1 if successful, 0 if the buffer is empty.
 */
static int ring_buf_pop(uint8_t *byte) {
  if (ring_head == ring_tail) {
    return 0;
  }
  *byte = rx_ring_buf[ring_tail];
  ring_tail = (ring_tail + 1) % RX_RING_BUF_SIZE;
  return 1;
}

/**
 * @brief Initializes the packet parser module.
 * @param huart UART peripheral handle used for receiving packets.
 */
void Packet_Parser_Init(UART_HandleTypeDef *huart) {
  target_huart = huart;
  ring_head = 0;
  ring_tail = 0;
}

/**
 * @brief Pushes a received byte into the ring buffer. Called inside ISR.
 * @param byte Incoming data byte.
 */
void Packet_Parser_Push_Byte(uint8_t byte) {
  uint16_t next_head = (ring_head + 1) % RX_RING_BUF_SIZE;
  if (next_head != ring_tail) {
    rx_ring_buf[ring_head] = byte;
    ring_head = next_head;
  }
}

/**
 * @brief Processes fully validated packets and updates control outputs.
 * @param cmd The command identifier.
 * @param payload Pointer to raw packet payload bytes.
 */
static void handle_valid_frame(uint8_t cmd, uint8_t *payload) {
  char dbg_buf[128];

  switch (cmd) {
  case 0x01: { // Speed Command: Updates sps targets
    Msg_SetSpeed_t *speed = (Msg_SetSpeed_t *)payload;

    // Apply speed updates only if emergency stop is not active
    if (!global_estop_flag) {
      global_left_sps = speed->left_sps;
      global_right_sps = speed->right_sps;
    }

    snprintf(dbg_buf, sizeof(dbg_buf),
             "[RCV SUCCESS] CMD: 0x01 | Left: %d sps, Right: %d sps\r\n",
             global_left_sps, global_right_sps);
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg_buf, strlen(dbg_buf), 50);
    break;
  }

  case 0x02: { // Nozzle Control Command
    Msg_ControlNozzle_t *nozzle = (Msg_ControlNozzle_t *)payload;

    global_nozzle_on = nozzle->nozzle_on;

    snprintf(dbg_buf, sizeof(dbg_buf),
             "[RCV SUCCESS] CMD: 0x02 | Nozzle: %s\r\n",
             global_nozzle_on ? "ON" : "OFF");
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg_buf, strlen(dbg_buf), 50);
    break;
  }
  case 0x03: { // Emergency Stop Command
    Msg_EStop_t *estop = (Msg_EStop_t *)payload;
    global_estop_flag = 1;

    // Safety Interlock: Force stop all motors and nozzle output
    global_left_sps = 0;
    global_right_sps = 0;
    global_nozzle_on = 0;

    snprintf(dbg_buf, sizeof(dbg_buf),
             "[RCV SUCCESS] CMD: 0x03 | E-Stop Reason: 0x%02X\r\n",
             estop->fault_reason);
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg_buf, strlen(dbg_buf), 100);
    break;
  }

  default:
    snprintf(dbg_buf, sizeof(dbg_buf), "[WARNING] Unknown CMD: 0x%02X\r\n",
             cmd);
    HAL_UART_Transmit(&huart2, (uint8_t *)dbg_buf, strlen(dbg_buf), 50);
    break;
  }
}

/**
 * @brief Non-blocking worker task. Parsers serial streams in the ring buffer.
 */
void Packet_Parser_Process(void) {
  static ParserState_t state = STATE_STX;
  static uint8_t packet_len = 0;
  static uint8_t packet_cmd = 0;
  static uint8_t packet_payload[RX_RING_BUF_SIZE];
  static uint8_t packet_payload_idx = 0;
  static uint8_t packet_crc = 0;

  uint8_t byte;
  while (ring_buf_pop(&byte)) {
    switch (state) {
    case STATE_STX:
      if (byte == 0xAA) {
        state = STATE_LEN;
      }
      break;
    case STATE_LEN:
      packet_len = byte;
      state = STATE_CMD;
      break;
    case STATE_CMD:
      packet_cmd = byte;
      packet_payload_idx = 0;
      if (packet_len > 0) {
        state = STATE_PAYLOAD;
      } else {
        state = STATE_CRC;
      }
      break;
    case STATE_PAYLOAD:
      packet_payload[packet_payload_idx++] = byte;
      if (packet_payload_idx >= packet_len) {
        state = STATE_CRC;
      }
      break;
    case STATE_CRC:
      packet_crc = byte;
      state = STATE_ETX;
      break;
    case STATE_ETX:
      if (byte == 0x55) {
        uint8_t calc_buf[RX_RING_BUF_SIZE + 2];
        calc_buf[0] = packet_len;
        calc_buf[1] = packet_cmd;
        if (packet_len > 0) {
          memcpy(&calc_buf[2], packet_payload, packet_len);
        }

        uint8_t calculated_crc = get_crc8(calc_buf, packet_len + 2);
        if (calculated_crc == packet_crc) {
          handle_valid_frame(packet_cmd, packet_payload);
        } else {
          char dbg_buf[64];
          snprintf(dbg_buf, sizeof(dbg_buf),
                   "[CRC ERR] Calc: 0x%02X, Recv: 0x%02X\r\n", calculated_crc,
                   packet_crc);
          HAL_UART_Transmit(&huart2, (uint8_t *)dbg_buf, strlen(dbg_buf), 50);
        }
      }
      state = STATE_STX;
      break;
    }
  }
}
