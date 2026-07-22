/**
 ******************************************************************************
 * @file    control_arbiter.c
 * @brief   ESTOP / IR 1회=고정 펄스 / UART SET_SPEED (모드 구분 없음)
 ******************************************************************************
 */

#include "control_arbiter.h"

#include "motor.h"
#include "robot_config.h"
#include "servo.h"
#include "usart.h"

#include <stdio.h>

typedef enum {
  IR_MOTION_IDLE = 0,
  IR_MOTION_PULSE
} IrMotion_t;

static volatile uint8_t estop_sources;
static uint8_t watchdog_armed;
static uint32_t last_set_speed_ms;

static IrMotion_t ir_motion;
static uint32_t pulse_start_left;
static uint32_t pulse_start_right;
static uint32_t pulse_target_steps;
static uint32_t pulse_deadline_ms;

static void apply_estop(uint8_t source_bit, uint8_t motor_reason,
                        uint8_t timeout) {
  estop_sources |= source_bit;
  Servo_SetNozzle(0U);
  Motor_RequestEStop(motor_reason, timeout);
  ir_motion = IR_MOTION_IDLE;
  watchdog_armed = 0U;
}

static uint8_t any_estop(void) {
  return (estop_sources != 0U) ? 1U : 0U;
}

static void debug_log_ir(uint8_t key, uint8_t accepted, const char *why) {
  char message[96];
  int length = snprintf(message, sizeof(message),
                        "[IR %s] key:0x%02X %s\r\n",
                        accepted ? "OK" : "REJECT", key,
                        (why != NULL) ? why : "");
  if (length > 0) {
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)message,
                            (uint16_t)length, 50U);
  }
}

static uint32_t pulse_progress_steps(const MotorSnapshot_t *motor) {
  int32_t left_delta = (int32_t)(motor->left_steps - pulse_start_left);
  int32_t right_delta = (int32_t)(motor->right_steps - pulse_start_right);
  uint32_t left_abs = (uint32_t)((left_delta < 0) ? -left_delta : left_delta);
  uint32_t right_abs =
      (uint32_t)((right_delta < 0) ? -right_delta : right_delta);
  return (left_abs + right_abs) / 2U;
}

static uint32_t pulse_stop_lead_steps(const MotorSnapshot_t *motor) {
  int16_t left = motor->current_left_sps;
  int16_t right = motor->current_right_sps;
  uint32_t v_left = (left < 0) ? (uint32_t)(-left) : (uint32_t)left;
  uint32_t v_right = (right < 0) ? (uint32_t)(-right) : (uint32_t)right;
  uint32_t v = (v_left > v_right) ? v_left : v_right;
  uint32_t lead;

  if (v == 0U || ROBOT_ACCEL_SPS2 == 0U) {
    return 0U;
  }

  lead = (v * v) / (2U * ROBOT_ACCEL_SPS2);
  lead += (v / 100U);
  return lead;
}

static void stop_ir_motion(void) {
  (void)Motor_SetTargetSps(0, 0);
  ir_motion = IR_MOTION_IDLE;
}

static void start_pulse_move(int16_t left_sps, int16_t right_sps,
                             uint32_t target_steps, uint32_t timeout_ms,
                             uint32_t now_ms, uint8_t key, const char *why) {
  MotorSnapshot_t motor;

  if (any_estop() != 0U) {
    return;
  }

  /* IR 펄스 중에는 서버 SET_SPEED watchdog이 끼어들지 않게 함 */
  watchdog_armed = 0U;
  stop_ir_motion();

  Motor_GetSnapshot(&motor);
  pulse_start_left = motor.left_steps;
  pulse_start_right = motor.right_steps;
  pulse_target_steps = target_steps;
  pulse_deadline_ms = now_ms + timeout_ms;

  if (Motor_SetTargetSps(left_sps, right_sps) != 0U) {
    ir_motion = IR_MOTION_PULSE;
    debug_log_ir(key, 1U, why);
  }
}

void ControlArbiter_Init(void) {
  estop_sources = 0U;
  watchdog_armed = 0U;
  last_set_speed_ms = 0U;
  ir_motion = IR_MOTION_IDLE;
  pulse_start_left = 0U;
  pulse_start_right = 0U;
  pulse_target_steps = 0U;
  pulse_deadline_ms = 0U;
}

uint8_t ControlArbiter_HandleUartFrame(const UartFrame_t *frame,
                                       uint32_t now_ms) {
  if (frame == NULL) {
    return 0U;
  }

  switch (frame->command) {
  case UART_CMD_SET_SPEED: {
    int16_t left_sps;
    int16_t right_sps;

    if (frame->length != UART_SET_SPEED_PAYLOAD_LEN) {
      return 0U;
    }
    if (any_estop() != 0U) {
      return 0U;
    }

    left_sps = UartProtocol_ReadI16Le(&frame->payload[0]);
    right_sps = UartProtocol_ReadI16Le(&frame->payload[2]);

    /*
     * (0,0) = 주차/유휴:
     *  - watchdog 해제 (끊어도 timeout ESTOP 안 남)
     *  - IR 펄스 중이면 무시 (계속 0,0 오면 리모컨이 바로 덮이는 것 방지)
     * 비제로 = 서버 주행 재개: IR 펄스 끊고 watchdog arm
     */
    if (left_sps == 0 && right_sps == 0) {
      if (ir_motion == IR_MOTION_PULSE) {
        return 1U;
      }
      ir_motion = IR_MOTION_IDLE;
      watchdog_armed = 0U;
      return Motor_SetTargetSps(0, 0);
    }

    ir_motion = IR_MOTION_IDLE;
    last_set_speed_ms = now_ms;
    watchdog_armed = 1U;
    return Motor_SetTargetSps(left_sps, right_sps);
  }

  case UART_CMD_NOZZLE: {
    MotorSnapshot_t motor;
    if (frame->length != UART_NOZZLE_PAYLOAD_LEN || frame->payload[0] > 1U) {
      return 0U;
    }
    Motor_GetSnapshot(&motor);
    /* SET_SPEED와 동일: ESTOP 중이면 노즐 명령도 거절 (이미 apply_estop에서 OFF) */
    if (any_estop() != 0U || motor.estop_latched != 0U) {
      return 0U;
    }
    Servo_SetNozzle(frame->payload[0] != 0U ? 1U : 0U);
    return 1U;
  }

  case UART_CMD_ESTOP:
    if (frame->length != UART_ESTOP_PAYLOAD_LEN) {
      return 0U;
    }
    apply_estop(ESTOP_SRC_SERVER,
                (frame->payload[0] != 0U) ? frame->payload[0]
                                          : ESTOP_REASON_RPI,
                0U);
    return 1U;

  case UART_CMD_CLEAR_ESTOP:
    if (frame->length != UART_CLEAR_ESTOP_PAYLOAD_LEN ||
        UartProtocol_ReadU16Le(frame->payload) != ROBOT_CLEAR_ESTOP_KEY) {
      return 0U;
    }
    Servo_SetNozzle(0U);
    if (Motor_ClearEStop() == 0U) {
      return 0U;
    }
    /*
     * 안전키(0xA55A) + 완전정지면 모든 ESTOP 원인 해제.
     * 예전에는 REMOTE를 남겨 VALID인데도 SET_SPEED가 계속 REJECT 됐음.
     */
    estop_sources = 0U;
    ir_motion = IR_MOTION_IDLE;
    watchdog_armed = 0U;
    return 1U;

  case UART_CMD_SET_CONTROL_MODE:
    /* 하위 호환: 예전 0x05는 받아만 주고 무시 */
    if (frame->length != UART_SET_CONTROL_MODE_PAYLOAD_LEN) {
      return 0U;
    }
    return 1U;

  case UART_CMD_STATUS:
  default:
    return 0U;
  }
}

uint8_t ControlArbiter_HandleIrEvent(const IrRemoteEvent_t *event,
                                     uint32_t now_ms) {
  if (event == NULL || event->valid == 0U) {
    return 0U;
  }

  IrRemote_NoteActivity(now_ms);

  if (event->key == IR_KEY_POWER) {
    if (event->is_repeat != 0U) {
      return 1U;
    }
    if ((estop_sources & ESTOP_SRC_REMOTE) != 0U) {
      Servo_SetNozzle(0U);
      /* 서버/timeout이 남아 있으면 REMOTE 비트만 해제 */
      if ((estop_sources & (uint8_t)~ESTOP_SRC_REMOTE) != 0U) {
        estop_sources &= (uint8_t)~ESTOP_SRC_REMOTE;
        debug_log_ir(event->key, 1U, "remote bit clear (other remains)");
        return 1U;
      }
      if (Motor_ClearEStop() == 0U) {
        /* REMOTE 비트 유지. 가짜 "clear"로 latch만 남는 상태 방지 */
        debug_log_ir(event->key, 0U, "remote clear FAIL not stopped");
        return 1U;
      }
      estop_sources &= (uint8_t)~ESTOP_SRC_REMOTE;
      debug_log_ir(event->key, 1U, "remote ESTOP clear");
    } else {
      apply_estop(ESTOP_SRC_REMOTE, ESTOP_REASON_REMOTE, 0U);
      debug_log_ir(event->key, 1U, "remote ESTOP");
    }
    return 1U;
  }

  if (any_estop() != 0U) {
    debug_log_ir(event->key, 0U, "ESTOP latched");
    return 0U;
  }
  if (event->is_repeat != 0U) {
    return 1U;
  }

  switch (event->key) {
  case IR_KEY_UP:
    start_pulse_move(ROBOT_MANUAL_DRIVE_SPS, ROBOT_MANUAL_DRIVE_SPS,
                     ROBOT_IR_FWD_STEPS, ROBOT_IR_FWD_MS, now_ms, event->key,
                     "fwd pulse");
    return 1U;
  case IR_KEY_DOWN:
    start_pulse_move((int16_t)(-ROBOT_MANUAL_DRIVE_SPS),
                     (int16_t)(-ROBOT_MANUAL_DRIVE_SPS), ROBOT_IR_REV_STEPS,
                     ROBOT_IR_REV_MS, now_ms, event->key, "rev pulse");
    return 1U;
  case IR_KEY_LEFT:
    start_pulse_move((int16_t)(-ROBOT_MANUAL_TURN_LEFT_SPS),
                     (int16_t)ROBOT_MANUAL_TURN_LEFT_SPS,
                     ROBOT_IR_TURN_90_STEPS_LEFT, ROBOT_IR_TURN_90_MS_LEFT,
                     now_ms, event->key, "turn L");
    return 1U;
  case IR_KEY_RIGHT:
    start_pulse_move((int16_t)ROBOT_MANUAL_TURN_RIGHT_SPS,
                     (int16_t)(-ROBOT_MANUAL_TURN_RIGHT_SPS),
                     ROBOT_IR_TURN_90_STEPS_RIGHT, ROBOT_IR_TURN_90_MS_RIGHT,
                     now_ms, event->key, "turn R");
    return 1U;
  case IR_KEY_NOZZLE_UP:
    Servo_SetNozzle(0U);
    debug_log_ir(event->key, 1U, "nozzle UP");
    return 1U;
  case IR_KEY_NOZZLE_DOWN:
    Servo_SetNozzle(1U);
    debug_log_ir(event->key, 1U, "nozzle DOWN");
    return 1U;
  default:
    debug_log_ir(event->key, 0U, "unknown");
    return 0U;
  }
}

void ControlArbiter_Service(uint32_t now_ms) {
  MotorSnapshot_t motor;

  if (ir_motion == IR_MOTION_PULSE) {
    uint32_t progress;
    uint32_t lead;

    Motor_GetSnapshot(&motor);
    progress = pulse_progress_steps(&motor);
    lead = pulse_stop_lead_steps(&motor);
    if ((progress + lead) >= pulse_target_steps ||
        (int32_t)(now_ms - pulse_deadline_ms) >= 0) {
      stop_ir_motion();
      debug_log_ir(0x00, 1U, "pulse stop");
    }
  }

  if (watchdog_armed == 0U) {
    return;
  }

  Motor_GetSnapshot(&motor);
  if (motor.estop_latched == 0U && any_estop() == 0U &&
      (uint32_t)(now_ms - last_set_speed_ms) >= ROBOT_UART_WATCHDOG_MS) {
    apply_estop(ESTOP_SRC_UART_TIMEOUT, ESTOP_REASON_UART_TIMEOUT, 1U);
  }
}

uint8_t ControlArbiter_GetEstopSources(void) {
  return estop_sources;
}

uint8_t ControlArbiter_IsMovementBlocked(void) {
  return any_estop();
}
