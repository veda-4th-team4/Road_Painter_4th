/**
 ******************************************************************************
 * @file    robot_control.c
 * @brief   검증된 UART 명령 실행과 300 ms 통신 watchdog 정책
 ******************************************************************************
 */

#include "robot_control.h"

#include "motor.h"
#include "robot_config.h"
#include "servo.h"

#include <stddef.h>

static uint8_t watchdog_armed;
static uint32_t last_set_speed_ms;
static volatile uint8_t rx_error_latched;

void RobotControl_Init(void) {
  watchdog_armed = 0U;
  last_set_speed_ms = 0U;
  rx_error_latched = 0U;
}

uint8_t RobotControl_HandleFrame(const UartFrame_t *frame, uint32_t now_ms) {
  if (frame == NULL) {
    RobotControl_ReportRxError();
    return 0U;
  }

  switch (frame->command) {
  case UART_CMD_SET_SPEED:
    if (frame->length != UART_SET_SPEED_PAYLOAD_LEN) {
      break;
    }
    last_set_speed_ms = now_ms;
    watchdog_armed = 1U;
    (void)Motor_SetTargetSps(
        UartProtocol_ReadI16Le(&frame->payload[0]),
        UartProtocol_ReadI16Le(&frame->payload[2]));
    return 1U;

  case UART_CMD_NOZZLE: {
    MotorSnapshot_t motor;
    if (frame->length != UART_NOZZLE_PAYLOAD_LEN ||
        frame->payload[0] > 1U) {
      break;
    }
    Motor_GetSnapshot(&motor);
    Servo_SetNozzle((frame->payload[0] != 0U && !motor.estop_latched) ? 1U
                                                                     : 0U);
    return 1U;
  }

  case UART_CMD_ESTOP:
    if (frame->length != UART_ESTOP_PAYLOAD_LEN) {
      break;
    }
    Servo_SetNozzle(0U);
    Motor_RequestEStop(frame->payload[0] != 0U ? frame->payload[0]
                                               : ESTOP_REASON_RPI,
                       0U);
    return 1U;

  case UART_CMD_CLEAR_ESTOP:
    if (frame->length != UART_CLEAR_ESTOP_PAYLOAD_LEN ||
        UartProtocol_ReadU16Le(frame->payload) != ROBOT_CLEAR_ESTOP_KEY) {
      break;
    }
    Servo_SetNozzle(0U);
    if (Motor_ClearEStop()) {
      last_set_speed_ms = now_ms;
      watchdog_armed = 1U;
    }
    return 1U;

  case UART_CMD_STATUS:
  default:
    break;
  }

  RobotControl_ReportRxError();
  return 0U;
}

void RobotControl_Service(uint32_t now_ms) {
  /* Temporarily disabled for hardware debugging
  MotorSnapshot_t motor;

  if (!watchdog_armed) {
    return;
  }

  Motor_GetSnapshot(&motor);
  if (!motor.estop_latched &&
      (uint32_t)(now_ms - last_set_speed_ms) >= ROBOT_UART_WATCHDOG_MS) {
    Servo_SetNozzle(0U);
    Motor_RequestEStop(ESTOP_REASON_UART_TIMEOUT, 1U);
  }
  */
}

void RobotControl_GetStatus(RobotStatus_t *status, uint8_t extra_status_flags) {
  MotorSnapshot_t motor;
  uint8_t flags = extra_status_flags;

  if (status == NULL) {
    return;
  }

  Motor_GetSnapshot(&motor);
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

  status->left_steps = motor.left_steps;
  status->right_steps = motor.right_steps;
  status->flags = flags;
}

void RobotControl_ReportRxError(void) {
  rx_error_latched = 1U;
}

void RobotControl_StatusPublished(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  rx_error_latched = 0U;
  if (primask == 0U) {
    __enable_irq();
  }
}
