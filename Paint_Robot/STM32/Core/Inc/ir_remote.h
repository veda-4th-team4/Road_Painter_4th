/**
 ******************************************************************************
 * @file    ir_remote.h
 * @brief   NEC IR 리모컨 디코더 (F103 예제 이식본)
 * @details TIM3 PWM-input 캡처로 리더/데이터/repeat를 해석합니다.
 *          키 매크로 값은 실제 리모컨에 맞게 수정하십시오.
 ******************************************************************************
 */

#ifndef INC_IR_REMOTE_H_
#define INC_IR_REMOTE_H_

#include "main.h"
#include <stdint.h>

/** @brief NEC 데이터 비트 '1' 판정용 전체 주기 임계값 [us]. */
#define IR_THRESHOLD 1700U

/** @brief 원본 예제 키 코드. 실제 보드에서 확인 후 필요 시 수정하십시오. */
#define IR_KEY_POWER 0x28U
#define IR_KEY_UP    0xC0U
#define IR_KEY_DOWN  0x40U
#define IR_KEY_RIGHT 0x58U
#define IR_KEY_LEFT  0x70U

/**
 * @brief 노즐 UP/DOWN 키 코드.
 * @note 실제 리모컨 값으로 교체하십시오. USART2에 `[IR Key]: 0xXX`가 출력됩니다.
 */
#define IR_KEY_NOZZLE_UP   0xD0U /* TODO(USER): 실제 키 코드로 교체 */
#define IR_KEY_NOZZLE_DOWN 0x1AU /* TODO(USER): 실제 키 코드로 교체 */

/** @brief ControlTask가 소비하는 IR 이벤트입니다. */
typedef struct {
  uint8_t key;     /**< 디코딩된 키 바이트. */
  uint8_t is_repeat; /**< NEC repeat이면 1, full frame이면 0. */
  uint8_t valid;   /**< 유효 이벤트이면 1. */
} IrRemoteEvent_t;

/**
 * @brief RAW 펄스 버퍼의 데이터 구역을 분석하여 키값을 반환합니다.
 * @param raw_buf 32개 비트의 전체 주기 [us] 배열입니다.
 * @param length 수집된 비트 수입니다.
 * @return 키 바이트, 불완전하면 0xFF입니다.
 */
uint8_t IR_Decode_Packet(volatile uint32_t *raw_buf, uint8_t length);

/**
 * @brief 디코딩 완료 시 내부 이벤트 큐에 키를 등록합니다.
 * @param key 디코딩된 키 바이트입니다.
 */
void IR_Complete_Callback(uint8_t key);

/**
 * @brief 타이머 캡처 인터럽트에서 호출되어 리모컨 신호를 수신·처리합니다.
 * @param htim 캡처가 발생한 타이머 핸들입니다.
 * @param huart 예제 호환용 인자이며 현재 미사용입니다.
 */
void IR_Handle_Interrupt(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart);

/** @brief IR 디코더 상태를 초기화합니다. */
void IrRemote_Init(void);

/**
 * @brief TIM3 PWM-input 캡처 interrupt를 시작합니다.
 * @return HAL_OK이면 성공입니다.
 */
HAL_StatusTypeDef IrRemote_Start(void);

/**
 * @brief ControlTask가 대기 중인 IR 이벤트 하나를 꺼냅니다.
 * @param[out] event 수신 이벤트입니다.
 * @return 이벤트가 있으면 1, 없으면 0입니다.
 */
uint8_t IrRemote_PollEvent(IrRemoteEvent_t *event);

/**
 * @brief 마지막으로 수신한 유효 키의 시각 [ms]을 반환합니다.
 * @note deadman 타이머용입니다. 수신이 없으면 0입니다.
 */
uint32_t IrRemote_LastActivityMs(void);

/**
 * @brief ControlTask가 현재 시각을 알려 deadman 판정에 사용합니다.
 * @param now_ms wrap-safe millisecond 시각입니다.
 */
void IrRemote_NoteActivity(uint32_t now_ms);

#endif /* INC_IR_REMOTE_H_ */
