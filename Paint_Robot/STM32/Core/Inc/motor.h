/**
 ******************************************************************************
 * @file    motor.h
 * @brief   DRV8825 기반 좌우 스텝모터 실시간 제어 인터페이스
 * @details TIM2 20 kHz 주기에서 Q16 속도 가감속, STEP 펄스 생성, 누적 스텝,
 *          ESTOP 상태를 관리합니다. 공개 제어 함수는 FreeRTOS task에서 호출하고
 *          Motor_TickISR()만 priority 0 TIM2 인터럽트에서 호출합니다.
 ******************************************************************************
 */

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "robot_config.h"

/**
 * @brief 모터 제어기의 일관된 상태 복사본입니다.
 * @details ISR과 메인 루프가 공유하는 값을 Motor_GetSnapshot()이 임계구역에서
 *          한 번에 복사하여 반환합니다.
 */
typedef struct {
  int16_t target_left_sps;  /**< 좌측 목표 속도 [microsteps/s]. */
  int16_t target_right_sps; /**< 우측 목표 속도 [microsteps/s]. */
  int16_t current_left_sps; /**< 좌측 현재 램프 속도 [microsteps/s]. */
  int16_t current_right_sps;/**< 우측 현재 램프 속도 [microsteps/s]. */
  uint32_t left_steps;      /**< 좌측 방향 포함 누적 스텝(modulo 2^32). */
  uint32_t right_steps;     /**< 우측 방향 포함 누적 스텝(modulo 2^32). */
  uint8_t moving;           /**< 목표 또는 현재 속도가 0이 아니면 1. */
  uint8_t estop_latched;    /**< ESTOP latch가 설정되어 있으면 1. */
  uint8_t timeout_latched;  /**< UART watchdog이 ESTOP 원인이면 1. */
  uint8_t estop_reason;     /**< 마지막 ESTOP 원인 코드. */
} MotorSnapshot_t;

/**
 * @brief GPIO 상태를 초기화하고 TIM2 20 kHz update interrupt를 시작합니다.
 * @return HAL_OK이면 초기화 성공, 그 외에는 TIM2 시작 실패입니다.
 * @pre MX_GPIO_Init()과 MX_TIM2_Init()이 먼저 완료되어야 합니다.
 */
HAL_StatusTypeDef Motor_Init(void);

/**
 * @brief 최신 좌/우 목표 속도를 설정합니다. 이전 목표는 즉시 대체됩니다.
 * @param left_sps 좌측 목표 속도 [microsteps/s], 양수는 전진입니다.
 * @param right_sps 우측 목표 속도 [microsteps/s], 양수는 전진입니다.
 * @return ESTOP latch 중이면 0, 명령을 수락하면 1입니다.
 * @note 입력은 내부에서 ROBOT_MAX_SPS 범위로 포화됩니다.
 * @warning ISR에서 호출하지 마십시오. ControlTask 전용 API입니다.
 */
uint8_t Motor_SetTargetSps(int16_t left_sps, int16_t right_sps);

/**
 * @brief 좌우 목표를 0으로 만들고 ESTOP 감속 및 latch를 요청합니다.
 * @param reason STATUS 진단에 보관할 ESTOP 원인 코드입니다.
 * @param timeout 1이면 통신 timeout latch도 함께 설정합니다.
 * @note 노즐 강제 OFF는 호출자가 먼저 수행합니다.
 * @warning ISR에서 호출하지 마십시오. ControlTask 전용 API입니다.
 */
void Motor_RequestEStop(uint8_t reason, uint8_t timeout);

/**
 * @brief 좌우 축이 완전히 정지한 경우에만 ESTOP latch를 해제합니다.
 * @return 해제했으면 1, 아직 움직이는 중이면 0입니다.
 */
uint8_t Motor_ClearEStop(void);

/**
 * @brief Q16 속도 램프와 좌우 STEP 펄스를 한 tick 진행합니다.
 * @warning TIM2_IRQHandler에서만 ROBOT_MOTOR_TICK_HZ 주기로 호출해야 합니다.
 */
void Motor_TickISR(void);

/**
 * @brief ISR 공유 상태를 원자적인 snapshot으로 반환합니다.
 * @param[out] snapshot 상태를 기록할 유효한 포인터입니다.
 * @note ControlTask와 CommunicationTask에서 호출할 수 있습니다.
 */
void Motor_GetSnapshot(MotorSnapshot_t *snapshot);

/**
 * @brief 치명 오류 시 STEP을 LOW로 만들고 두 드라이버를 즉시 차단합니다.
 * @note 정상 감속을 생략하는 Error_Handler 전용 fail-safe 함수입니다.
 */
void Motor_ForceDisable(void);

#endif /* __MOTOR_H__ */
