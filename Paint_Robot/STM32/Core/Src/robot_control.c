/**
 ******************************************************************************
 * @file    robot_control.c
 * @brief   검증된 UART 명령 실행과 arbiter 위임
 ******************************************************************************
 */

#include "robot_control.h"

#include "control_arbiter.h"
#include "motor.h"
#include "robot_config.h"
#include "servo.h"

#include <stddef.h>

static volatile uint8_t rx_error_latched;

void RobotControl_Init(void) {
  rx_error_latched = 0U;
  ControlArbiter_Init();
}

uint8_t RobotControl_HandleFrame(const UartFrame_t *frame, uint32_t now_ms) {
  uint8_t handled;

  if (frame == NULL) {
    RobotControl_ReportRxError();
    return 0U;
  }

  handled = ControlArbiter_HandleUartFrame(frame, now_ms);
  if (handled == 0U && frame->command != UART_CMD_STATUS) {
    RobotControl_ReportRxError();
  }
  return handled;
}

void RobotControl_Service(uint32_t now_ms) {
  ControlArbiter_Service(now_ms);
}

void RobotControl_GetStatus(RobotStatus_t *status, uint8_t extra_status_flags) {
  MotorSnapshot_t motor;
  uint8_t flags = extra_status_flags;
  uint8_t sources;

  if (status == NULL) {
    return;
  }

  Motor_GetSnapshot(&motor);
  sources = ControlArbiter_GetEstopSources();

  if (motor.moving) {
    flags |= STATUS_FLAG_MOVING;
  }
  if (motor.estop_latched || sources != 0U) {
    flags |= STATUS_FLAG_ESTOP;
  }
  if (motor.timeout_latched ||
      (sources & ESTOP_SRC_UART_TIMEOUT) != 0U) {
    flags |= STATUS_FLAG_TIMEOUT;
  }
  if (Servo_IsNozzleOn()) {
    flags |= STATUS_FLAG_NOZZLE;
  }
  if (rx_error_latched) {
    flags |= STATUS_FLAG_RX_ERROR;
  }
  /* bit6 MANUAL: 모드 구분 폐지로 항상 0 */

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

uint8_t RobotControl_HandleIrEvent(const IrRemoteEvent_t *event,
                                   uint32_t now_ms) {
  return ControlArbiter_HandleIrEvent(event, now_ms);
}
