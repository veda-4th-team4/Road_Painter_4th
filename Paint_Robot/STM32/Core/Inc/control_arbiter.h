/**
 ******************************************************************************
 * @file    control_arbiter.h
 * @brief   ESTOP 중재 및 IR 펄스 / UART SET_SPEED 실행
 * @note    AUTO/MANUAL 모드 구분 없음. 서버와 IR 모두 ESTOP이 아니면 사용 가능.
 * @note    0x04 CLEAR_ESTOP(키 0xA55A)은 완전정지 시 SERVER/TIMEOUT/REMOTE를 모두 해제합니다.
 *          서버 주행 중에는 리모컨을 누르지 않으면 됩니다.
 ******************************************************************************
 */

#ifndef __CONTROL_ARBITER_H__
#define __CONTROL_ARBITER_H__

#include "ir_remote.h"
#include "uart_protocol.h"

#include <stdint.h>

/** @brief ESTOP 원인 bitmask입니다. 하나라도 남으면 이동이 금지됩니다. */
#define ESTOP_SRC_SERVER       (1U << 0)
#define ESTOP_SRC_REMOTE       (1U << 1)
#define ESTOP_SRC_UART_TIMEOUT (1U << 2)

void ControlArbiter_Init(void);

/**
 * @brief UART 프레임을 정책에 따라 실행합니다.
 * @return 수락하면 1, 거부하면 0입니다.
 */
uint8_t ControlArbiter_HandleUartFrame(const UartFrame_t *frame,
                                       uint32_t now_ms);

/**
 * @brief IR 이벤트를 정책에 따라 실행합니다.
 * @return 수락하면 1, 거부/무시하면 0입니다.
 */
uint8_t ControlArbiter_HandleIrEvent(const IrRemoteEvent_t *event,
                                     uint32_t now_ms);

/**
 * @brief IR 펄스 종료와 UART watchdog을 검사합니다.
 */
void ControlArbiter_Service(uint32_t now_ms);

uint8_t ControlArbiter_GetEstopSources(void);
uint8_t ControlArbiter_IsMovementBlocked(void);

#endif /* __CONTROL_ARBITER_H__ */
