/**
 ******************************************************************************
 * @file    app_rtos.c
 * @brief   정적 FreeRTOS task/queue 생성과 모듈 간 data flow
 ******************************************************************************
 */

#include "app_rtos.h"

#include "FreeRTOS.h"
#include "ir_remote.h"
#include "queue.h"
#include "robot_config.h"
#include "robot_control.h"
#include "task.h"
#include "uart_protocol.h"
#include "uart_transport.h"
#include "usart.h"

#include <stddef.h>
#include <stdio.h>

#define COMMAND_QUEUE_DEPTH       8U
#define COMM_TASK_STACK_WORDS     768U
#define CONTROL_TASK_STACK_WORDS  512U
#define COMM_TASK_PRIORITY        (tskIDLE_PRIORITY + 2U)
#define CONTROL_TASK_PRIORITY     (tskIDLE_PRIORITY + 3U)
#define SERVICE_PERIOD_MS         10U
#define RX_CHUNK_SIZE             32U

static StaticQueue_t command_queue_control;
static uint8_t command_queue_storage[COMMAND_QUEUE_DEPTH * sizeof(UartFrame_t)];
static QueueHandle_t command_queue;

static StaticTask_t communication_task_tcb;
static StackType_t communication_task_stack[COMM_TASK_STACK_WORDS];
static TaskHandle_t communication_task_handle;

static StaticTask_t control_task_tcb;
static StackType_t control_task_stack[CONTROL_TASK_STACK_WORDS];
static TaskHandle_t control_task_handle;
static uint8_t pending_transport_errors;

/** @brief FreeRTOS tick을 wrap-safe millisecond 값으로 변환합니다. */
static uint32_t ticks_to_ms(TickType_t ticks) {
  return (uint32_t)ticks * (uint32_t)portTICK_PERIOD_MS;
}

/**
 * @brief 검증 후 ControlTask까지 전달된 명령을 USART2 진단 포트에 출력합니다.
 */
static void debug_log_command(const UartFrame_t *frame, uint8_t handled) {
  char message[128];
  int length;

  if (frame == NULL) {
    return;
  }

  if (!handled) {
    if (frame->command == UART_CMD_STATUS) {
      return;
    }
    length = snprintf(message, sizeof(message),
                      "[RCV REJECT] CMD: 0x%02X | LEN: %u\r\n",
                      frame->command, frame->length);
  } else {
    switch (frame->command) {
    case UART_CMD_SET_SPEED:
      length = snprintf(
          message, sizeof(message),
          "[RCV VALID] CMD: 0x01 | Left: %d sps, Right: %d sps\r\n",
          (int)UartProtocol_ReadI16Le(&frame->payload[0]),
          (int)UartProtocol_ReadI16Le(&frame->payload[2]));
      break;

    case UART_CMD_NOZZLE:
      length = snprintf(message, sizeof(message),
                        "[RCV VALID] CMD: 0x02 | Nozzle Request: %s\r\n",
                        frame->payload[0] ? "ON" : "OFF");
      break;

    case UART_CMD_ESTOP:
      length = snprintf(message, sizeof(message),
                        "[RCV VALID] CMD: 0x03 | E-Stop Reason: 0x%02X\r\n",
                        frame->payload[0]);
      break;

    case UART_CMD_CLEAR_ESTOP:
      length = snprintf(message, sizeof(message),
                        "[RCV VALID] CMD: 0x04 | Clear E-Stop Request\r\n");
      break;

    case UART_CMD_SET_CONTROL_MODE:
      length = snprintf(message, sizeof(message),
                        "[RCV VALID] CMD: 0x05 | ignored (no mode)\r\n");
      break;

    default:
      length = snprintf(message, sizeof(message),
                        "[RCV VALID] CMD: 0x%02X | LEN: %u\r\n",
                        frame->command, frame->length);
      break;
    }
  }

  if (length > 0) {
    uint16_t tx_length =
        (uint16_t)((length < (int)sizeof(message)) ? length
                                                   : (int)sizeof(message) - 1);
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message, tx_length, 50U);
  }
}

/** @brief ESTOP을 우선 전달하고 일반 명령을 FIFO로 전달합니다. */
static void queue_command(const UartFrame_t *frame) {
  BaseType_t queued;

  if (frame->command == UART_CMD_ESTOP) {
    queued = xQueueSendToFront(command_queue, frame, 0U);
    if (queued != pdPASS) {
      UartFrame_t discarded;
      (void)xQueueReceive(command_queue, &discarded, 0U);
      queued = xQueueSendToFront(command_queue, frame, 0U);
    }
  } else {
    queued = xQueueSendToBack(command_queue, frame, 0U);
  }

  if (queued != pdPASS) {
    RobotControl_ReportRxError();
  }
}

/** @brief 현재 도메인 상태를 기존 0x81 wire frame으로 송신합니다. */
static void publish_status(void) {
  RobotStatus_t status;
  uint8_t payload[UART_STATUS_PAYLOAD_LEN];
  uint8_t frame[UART_PROTOCOL_MAX_FRAME];
  uint8_t frame_length;
  uint8_t extra_flags = 0U;

  if ((pending_transport_errors & UART_TRANSPORT_ERROR_TX_OVERFLOW) != 0U) {
    extra_flags |= STATUS_FLAG_TX_OVERFLOW;
  }

  RobotControl_GetStatus(&status, extra_flags);
  UartProtocol_WriteU32Le(&payload[0], status.left_steps);
  UartProtocol_WriteU32Le(&payload[4], status.right_steps);
  payload[8] = status.flags;

  if (UartProtocol_EncodeFrame(UART_CMD_STATUS, payload, sizeof(payload), frame,
                               sizeof(frame), &frame_length) &&
      UartTransport_Send(frame, frame_length)) {
    RobotControl_StatusPublished();
    pending_transport_errors = 0U;
  }
}

/** @brief UART byte stream을 frame으로 변환하고 STATUS를 주기 송신합니다. */
static void CommunicationTask(void *argument) {
  UartParser_t parser;
  UartFrame_t completed_frame;
  uint8_t rx_bytes[RX_CHUNK_SIZE];
  TickType_t last_status_tick;
  const TickType_t service_period = pdMS_TO_TICKS(SERVICE_PERIOD_MS);
  const TickType_t status_period = pdMS_TO_TICKS(ROBOT_STATUS_PERIOD_MS);

  (void)argument;
  UartProtocol_ParserInit(&parser);
  if (UartTransport_Start() != HAL_OK) {
    Error_Handler();
  }
  last_status_tick = xTaskGetTickCount();

  for (;;) {
    size_t received =
        UartTransport_Read(rx_bytes, sizeof(rx_bytes), service_period);
    uint8_t new_transport_errors = UartTransport_TakeErrorFlags();
    pending_transport_errors |= new_transport_errors;
    if ((new_transport_errors & UART_TRANSPORT_ERROR_RX) != 0U) {
      UartTransport_DiscardRx();
      UartProtocol_ParserInit(&parser);
      RobotControl_ReportRxError();
      received = 0U;
    }

    for (size_t index = 0U; index < received; index++) {
      UartParseResult_t result =
          UartProtocol_ParseByte(&parser, rx_bytes[index], &completed_frame);
      if (result == UART_PARSE_FRAME_READY) {
        /* 0x81 STATUS는 STM32→RPi 전용. RX로 들어오면(에코/오배선) 무시 */
        if (completed_frame.command != UART_CMD_STATUS) {
          queue_command(&completed_frame);
        }
      } else if (result == UART_PARSE_ERROR) {
        RobotControl_ReportRxError();
      }
    }

    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - last_status_tick) >= status_period) {
      last_status_tick = now;
      publish_status();
    }
  }
}

/**
 * @brief UART 명령, IR 이벤트, deadman/watchdog를 실행합니다.
 */
static void ControlTask(void *argument) {
  UartFrame_t frame;
  IrRemoteEvent_t ir_event;
  const TickType_t service_period = pdMS_TO_TICKS(SERVICE_PERIOD_MS);

  (void)argument;
  for (;;) {
    if (xQueueReceive(command_queue, &frame, service_period) == pdPASS) {
      uint8_t handled = RobotControl_HandleFrame(
          &frame, ticks_to_ms(xTaskGetTickCount()));
      debug_log_command(&frame, handled);
    }

    while (IrRemote_PollEvent(&ir_event) != 0U) {
      char ir_msg[48];
      int ir_len = snprintf(
          ir_msg, sizeof(ir_msg), "\r\n[IR Key]: 0x%02X%s\r\n", ir_event.key,
          ir_event.is_repeat ? " (rpt)" : "");
      if (ir_len > 0) {
        (void)HAL_UART_Transmit(&huart2, (uint8_t *)ir_msg, (uint16_t)ir_len,
                                50U);
      }
      (void)RobotControl_HandleIrEvent(&ir_event,
                                       ticks_to_ms(xTaskGetTickCount()));
    }

    RobotControl_Service(ticks_to_ms(xTaskGetTickCount()));
  }
}

HAL_StatusTypeDef AppRtos_Init(UART_HandleTypeDef *command_uart) {
  command_queue = xQueueCreateStatic(
      COMMAND_QUEUE_DEPTH, sizeof(UartFrame_t), command_queue_storage,
      &command_queue_control);
  if (command_queue == NULL ||
      UartTransport_Init(command_uart) != HAL_OK) {
    return HAL_ERROR;
  }

  IrRemote_Init();
  if (IrRemote_Start() != HAL_OK) {
    return HAL_ERROR;
  }

  RobotControl_Init();
  pending_transport_errors = 0U;
  communication_task_handle = xTaskCreateStatic(
      CommunicationTask, "Communication", COMM_TASK_STACK_WORDS, NULL,
      COMM_TASK_PRIORITY, communication_task_stack, &communication_task_tcb);
  control_task_handle = xTaskCreateStatic(
      ControlTask, "Control", CONTROL_TASK_STACK_WORDS, NULL,
      CONTROL_TASK_PRIORITY, control_task_stack, &control_task_tcb);

  return (communication_task_handle != NULL && control_task_handle != NULL)
             ? HAL_OK
             : HAL_ERROR;
}

void AppRtos_GetStackWatermarks(AppRtosStackWatermark_t *watermarks) {
  if (watermarks == NULL || communication_task_handle == NULL ||
      control_task_handle == NULL) {
    return;
  }

  watermarks->communication_free_words =
      (uint32_t)uxTaskGetStackHighWaterMark(communication_task_handle);
  watermarks->control_free_words =
      (uint32_t)uxTaskGetStackHighWaterMark(control_task_handle);
}
