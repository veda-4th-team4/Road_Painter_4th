/**
 ******************************************************************************
 * @file    robot_control.h
 * @brief   RTOS 비종속 로봇 명령·안전 정책 서비스
 ******************************************************************************
 */

#ifndef __ROBOT_CONTROL_H__
#define __ROBOT_CONTROL_H__

#include "ir_remote.h"
#include "uart_protocol.h"

#include <stdint.h>

/** @brief 0x81 STATUS 생성에 필요한 도메인 상태입니다. */
typedef struct {
  uint32_t left_steps;
  uint32_t right_steps;
  uint8_t flags;
} RobotStatus_t;

void RobotControl_Init(void);
uint8_t RobotControl_HandleFrame(const UartFrame_t *frame, uint32_t now_ms);
uint8_t RobotControl_HandleIrEvent(const IrRemoteEvent_t *event,
                                   uint32_t now_ms);
void RobotControl_Service(uint32_t now_ms);
void RobotControl_GetStatus(RobotStatus_t *status, uint8_t extra_status_flags);
void RobotControl_ReportRxError(void);
void RobotControl_StatusPublished(void);

#endif /* __ROBOT_CONTROL_H__ */
