#include "uart_protocol.h"
#include "motor.h"
#include "servo.h"
#include <string.h>

/**
 ******************************************************************************
 * V-[HW] UART 통신 정본:
 * [0xAA][LEN][CMD][PAYLOAD...][CRC8][0x55]
 * LEN은 payload 길이, CRC 범위는 LEN부터 payload 끝, Little-Endian.
 ******************************************************************************
 */

typedef enum {
  PARSER_WAIT_STX,
  PARSER_LEN,
  PARSER_CMD,
  PARSER_PAYLOAD,
  PARSER_CRC,
  PARSER_ETX
} ParserState_t;

typedef struct {
  uint8_t bytes[ROBOT_UART_MAX_FRAME];
  uint8_t length;
} TxFrame_t;

static UART_HandleTypeDef *cmd_uart;
static uint8_t rx_byte;
static uint8_t rx_ring[ROBOT_UART_RX_RING_SIZE];
static volatile uint16_t rx_head;
static volatile uint16_t rx_tail;
static volatile uint8_t rx_overflow;

static TxFrame_t tx_queue[ROBOT_UART_TX_QUEUE_DEPTH];
static volatile uint8_t tx_head;
static volatile uint8_t tx_tail;
static volatile uint8_t tx_busy;
static volatile uint8_t tx_overflow;

static ParserState_t parser_state;
static uint8_t parser_len;
static uint8_t parser_cmd;
static uint8_t parser_payload[ROBOT_UART_MAX_PAYLOAD];
static uint8_t parser_index;
static uint8_t parser_crc;
static uint8_t parser_received_crc;

static volatile uint8_t rx_error_latched;
static uint8_t watchdog_armed;
static uint32_t last_set_speed_ms;
static uint32_t last_status_ms;
static uint32_t protocol_now_ms;

static uint8_t crc8_update(uint8_t crc, uint8_t data) {
  crc ^= data;
  for (uint8_t bit = 0U; bit < 8U; bit++) {
    crc = (crc & 0x80U) ? (uint8_t)((crc << 1) ^ 0x07U)
                        : (uint8_t)(crc << 1);
  }
  return crc;
}

uint8_t UartProtocol_Crc8(const uint8_t *data, uint16_t len) {
  uint8_t crc = 0U;
  if (data == NULL) {
    return crc;
  }
  for (uint16_t i = 0U; i < len; i++) {
    crc = crc8_update(crc, data[i]);
  }
  return crc;
}

static void parser_reset(void) {
  parser_state = PARSER_WAIT_STX;
  parser_len = 0U;
  parser_cmd = 0U;
  parser_index = 0U;
  parser_crc = 0U;
  parser_received_crc = 0U;
}

static int16_t read_i16_le(const uint8_t *data) {
  return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static uint16_t read_u16_le(const uint8_t *data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static void write_u32_le(uint8_t *data, uint32_t value) {
  data[0] = (uint8_t)value;
  data[1] = (uint8_t)(value >> 8);
  data[2] = (uint8_t)(value >> 16);
  data[3] = (uint8_t)(value >> 24);
}

static HAL_StatusTypeDef tx_start_current(void) {
  if (!tx_busy || tx_tail == tx_head) {
    return HAL_OK;
  }
  HAL_StatusTypeDef result =
      HAL_UART_Transmit_IT(cmd_uart, tx_queue[tx_tail].bytes,
                           tx_queue[tx_tail].length);
  if (result != HAL_OK) {
    tx_busy = 0U;
    tx_overflow = 1U;
  }
  return result;
}

static uint8_t queue_frame(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  if (len > ROBOT_UART_MAX_PAYLOAD) {
    return 0U;
  }

  uint8_t frame[ROBOT_UART_MAX_FRAME];
  frame[0] = UART_FRAME_STX;
  frame[1] = len;
  frame[2] = cmd;
  if (len > 0U && payload != NULL) {
    memcpy(&frame[3], payload, len);
  }
  frame[3U + len] = UartProtocol_Crc8(&frame[1], (uint16_t)len + 2U);
  frame[4U + len] = UART_FRAME_ETX;

  uint8_t should_start = 0U;
  __disable_irq();
  uint8_t next = (uint8_t)((tx_head + 1U) % ROBOT_UART_TX_QUEUE_DEPTH);
  if (next == tx_tail) {
    tx_overflow = 1U;
    __enable_irq();
    return 0U;
  }
  memcpy(tx_queue[tx_head].bytes, frame, (uint16_t)len + 5U);
  tx_queue[tx_head].length = (uint8_t)(len + 5U);
  tx_head = next;
  if (!tx_busy) {
    tx_busy = 1U;
    should_start = 1U;
  }
  __enable_irq();

  if (should_start) {
    (void)tx_start_current();
  }
  return 1U;
}

static void send_status(void) {
  MotorSnapshot_t motor;
  uint8_t payload[9];
  uint8_t flags = 0U;

  Motor_GetSnapshot(&motor);
  write_u32_le(&payload[0], motor.left_steps);
  write_u32_le(&payload[4], motor.right_steps);
  if (motor.moving) {
    flags |= STATUS_FLAG_MOVING;
  }
  if (motor.estop_latched) {
    flags |= STATUS_FLAG_ESTOP;
  }
  if (motor.timeout_latched) {
    flags |= STATUS_FLAG_TIMEOUT;
  }
  if (Servo_IsNozzleOn()) {
    flags |= STATUS_FLAG_NOZZLE;
  }
  if (rx_error_latched) {
    flags |= STATUS_FLAG_RX_ERROR;
  }
  if (tx_overflow) {
    flags |= STATUS_FLAG_TX_OVERFLOW;
  }
  payload[8] = flags;

  if (queue_frame(UART_CMD_STATUS, payload, sizeof(payload))) {
    rx_error_latched = 0U;
    tx_overflow = 0U;
  }
}

static void dispatch_frame(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  switch (cmd) {
  case 0x01: { /* SET_SPEED: int16 left_sps, int16 right_sps */
    if (len != 4U) {
      rx_error_latched = 1U;
      break;
    }
    int16_t left_sps = read_i16_le(&payload[0]);
    int16_t right_sps = read_i16_le(&payload[2]);
    last_set_speed_ms = protocol_now_ms;
    watchdog_armed = 1U;
    (void)Motor_SetTargetSps(left_sps, right_sps);
    break;
  }

  case 0x02: { /* NOZZLE: uint8 on */
    if (len != 1U || payload[0] > 1U) {
      rx_error_latched = 1U;
      break;
    }
    MotorSnapshot_t nozzle_motor;
    Motor_GetSnapshot(&nozzle_motor);
    if (payload[0] && nozzle_motor.estop_latched) {
      /* ESTOP latch 중에는 노즐 ON을 절대 허용하지 않습니다. */
      Servo_SetNozzle(0U);
    } else {
      Servo_SetNozzle(payload[0]);
    }
    break;
  }

  case 0x03: /* ESTOP: uint8 reason */
    if (len != 1U) {
      rx_error_latched = 1U;
      break;
    }
    Servo_SetNozzle(0U);
    Motor_RequestEStop(payload[0] ? payload[0] : ESTOP_REASON_RPI, 0U);
    break;

  case 0x04: { /* CLEAR_ESTOP: uint16 safety key */
    if (len != 2U || read_u16_le(payload) != ROBOT_CLEAR_ESTOP_KEY) {
      rx_error_latched = 1U;
      break;
    }
    Servo_SetNozzle(0U);
    if (Motor_ClearEStop()) {
      last_set_speed_ms = protocol_now_ms;
      watchdog_armed = 1U;
    }
    break;
  }

  case 0x81: /* STATUS는 STM32->RPi 전용 */
  default:
    rx_error_latched = 1U;
    break;
  }
}

static void parser_push(uint8_t byte) {
  switch (parser_state) {
  case PARSER_WAIT_STX:
    if (byte == UART_FRAME_STX) {
      parser_state = PARSER_LEN;
    }
    break;

  case PARSER_LEN:
    if (byte > ROBOT_UART_MAX_PAYLOAD) {
      rx_error_latched = 1U;
      parser_reset();
      break;
    }
    parser_len = byte;
    parser_index = 0U;
    parser_crc = crc8_update(0U, byte);
    parser_state = PARSER_CMD;
    break;

  case PARSER_CMD:
    parser_cmd = byte;
    parser_crc = crc8_update(parser_crc, byte);
    parser_state = (parser_len == 0U) ? PARSER_CRC : PARSER_PAYLOAD;
    break;

  case PARSER_PAYLOAD:
    parser_payload[parser_index++] = byte;
    parser_crc = crc8_update(parser_crc, byte);
    if (parser_index >= parser_len) {
      parser_state = PARSER_CRC;
    }
    break;

  case PARSER_CRC:
    parser_received_crc = byte;
    parser_state = PARSER_ETX;
    break;

  case PARSER_ETX:
    if (byte == UART_FRAME_ETX && parser_received_crc == parser_crc) {
      dispatch_frame(parser_cmd, parser_payload, parser_len);
      parser_reset();
    } else {
      rx_error_latched = 1U;
      parser_reset();
      if (byte == UART_FRAME_STX) {
        parser_state = PARSER_LEN;
      }
    }
    break;
  }
}

HAL_StatusTypeDef UartProtocol_Init(UART_HandleTypeDef *command_uart,
                                    UART_HandleTypeDef *debug_uart) {
  cmd_uart = command_uart;
  (void)debug_uart; /* USART2는 main.c의 사람이 읽는 부팅 로그 전용입니다. */
  rx_head = rx_tail = 0U;
  tx_head = tx_tail = 0U;
  rx_overflow = tx_busy = tx_overflow = 0U;
  rx_error_latched = watchdog_armed = 0U;
  last_set_speed_ms = last_status_ms = protocol_now_ms = HAL_GetTick();
  parser_reset();

  if (cmd_uart == NULL) {
    return HAL_ERROR;
  }
  return HAL_UART_Receive_IT(cmd_uart, &rx_byte, 1U);
}

void UartProtocol_Process(uint32_t now_ms) {
  protocol_now_ms = now_ms;

  if (rx_overflow) {
    __disable_irq();
    rx_tail = rx_head;
    rx_overflow = 0U;
    __enable_irq();
    parser_reset();
    rx_error_latched = 1U;
  }

  while (rx_tail != rx_head) {
    uint8_t byte = rx_ring[rx_tail];
    rx_tail = (uint16_t)((rx_tail + 1U) % ROBOT_UART_RX_RING_SIZE);
    parser_push(byte);
  }

  MotorSnapshot_t motor;
  Motor_GetSnapshot(&motor);
  if (watchdog_armed && !motor.estop_latched &&
      (uint32_t)(now_ms - last_set_speed_ms) >= ROBOT_UART_WATCHDOG_MS) {
    Servo_SetNozzle(0U);
    Motor_RequestEStop(ESTOP_REASON_UART_TIMEOUT, 1U);
  }

  if ((uint32_t)(now_ms - last_status_ms) >= ROBOT_STATUS_PERIOD_MS) {
    last_status_ms = now_ms;
    send_status();
  }

  if (!tx_busy && tx_tail != tx_head) {
    tx_busy = 1U;
    (void)tx_start_current();
  }
}

void UartProtocol_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (cmd_uart == NULL || huart->Instance != cmd_uart->Instance) {
    return;
  }
  uint16_t next = (uint16_t)((rx_head + 1U) % ROBOT_UART_RX_RING_SIZE);
  if (next == rx_tail) {
    rx_overflow = 1U;
  } else {
    rx_ring[rx_head] = rx_byte;
    rx_head = next;
  }
  if (HAL_UART_Receive_IT(cmd_uart, &rx_byte, 1U) != HAL_OK) {
    rx_error_latched = 1U;
  }
}

void UartProtocol_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (cmd_uart == NULL || huart->Instance != cmd_uart->Instance) {
    return;
  }
  tx_tail = (uint8_t)((tx_tail + 1U) % ROBOT_UART_TX_QUEUE_DEPTH);
  if (tx_tail != tx_head) {
    tx_busy = 1U;
    (void)tx_start_current();
  } else {
    tx_busy = 0U;
  }
}

void UartProtocol_ErrorCallback(UART_HandleTypeDef *huart) {
  if (cmd_uart == NULL || huart->Instance != cmd_uart->Instance) {
    return;
  }
  rx_error_latched = 1U;
  __HAL_UART_CLEAR_OREFLAG(huart);
  if (huart->RxState == HAL_UART_STATE_READY) {
    (void)HAL_UART_Receive_IT(cmd_uart, &rx_byte, 1U);
  }
}
