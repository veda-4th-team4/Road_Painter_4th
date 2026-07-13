#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "robot_config.h"

typedef struct {
  int16_t target_left_sps;
  int16_t target_right_sps;
  int16_t current_left_sps;
  int16_t current_right_sps;
  uint32_t left_steps;
  uint32_t right_steps;
  uint8_t moving;
  uint8_t estop_latched;
  uint8_t timeout_latched;
  uint8_t estop_reason;
} MotorSnapshot_t;

/**
 * @brief GPIO 상태를 초기화하고 TIM2 20 kHz update interrupt를 시작합니다.
 */
HAL_StatusTypeDef Motor_Init(void);

/**
 * @brief 최신 좌/우 목표 속도를 설정합니다. 이전 목표는 즉시 대체됩니다.
 * @return ESTOP latch 중이면 0, 수락하면 1
 */
uint8_t Motor_SetTargetSps(int16_t left_sps, int16_t right_sps);

/* 감속 정지 후 latch. timeout=1이면 STATUS TIMEOUT flag도 설정합니다. */
void Motor_RequestEStop(uint8_t reason, uint8_t timeout);

/* 완전 정지 상태에서만 ESTOP latch를 해제합니다. */
uint8_t Motor_ClearEStop(void);

/* TIM2_IRQHandler에서만 호출하는 20 kHz 실시간 루프입니다. */
void Motor_TickISR(void);

/* ISR 공유 상태를 atomic snapshot으로 반환합니다. */
void Motor_GetSnapshot(MotorSnapshot_t *snapshot);

/* 치명 오류용: 펄스/드라이버를 즉시 하드웨어 차단합니다. */
void Motor_ForceDisable(void);

#endif /* __MOTOR_H__ */
