/**
 ******************************************************************************
 * @file    ir_remote.c
 * @brief   NEC IR 리모컨 수신기 구현 (F103 예제 구조 유지)
 ******************************************************************************
 */

#include "ir_remote.h"

#include "tim.h"

#include <stdio.h>

#define IR_EVENT_QUEUE_DEPTH 8U
#define IR_LEADER_PERIOD_US  8000U
#define IR_LEADER_HIGH_US    4000U
#define IR_REPEAT_HIGH_MIN_US 1500U
#define IR_REPEAT_HIGH_MAX_US 4000U

/** @brief 리모컨 수신을 위한 내부 변수 (캡슐화) */
static uint32_t ir_raw_buf[32];
static uint8_t ir_idx = 0;

static volatile uint8_t last_key = 0xFFU;
static volatile uint8_t have_last_key = 0U;
static volatile uint32_t last_activity_ms;

static IrRemoteEvent_t event_queue[IR_EVENT_QUEUE_DEPTH];
static volatile uint8_t event_head;
static volatile uint8_t event_tail;

/**
 * @brief RAW 펄스 버퍼의 데이터 구역을 분석하여 중복 연산 없이 키값을 반환합니다.
 */
uint8_t IR_Decode_Packet(volatile uint32_t *raw_buf, uint8_t length) {
  /* 최소 32개 비트 완결성 확인 */
  if (length < 32U) {
    return 0xFFU;
  }

  uint8_t key_byte = 0U;

  /*
   * 배열의 [16]번 부터 [23]번 까지가 실제 리모컨의 데이터 바이트 영역입니다.
   * 다른 비트는 검사할 필요도 없이, 딱 이 8개 방만 돌면서 8비트 변수에 밀어 넣습니다.
   */
  for (int i = 0; i < 8; i++) {
    if (raw_buf[16 + i] > IR_THRESHOLD) {
      key_byte |= (uint8_t)(1U << (7 - i)); /* 배열 순서대로 상위 비트부터 적재 */
    }
  }

  return key_byte;
}

static void push_event(uint8_t key, uint8_t is_repeat) {
  uint8_t next = (uint8_t)((event_head + 1U) % IR_EVENT_QUEUE_DEPTH);
  if (next == event_tail) {
    /* 오래된 이벤트를 버리고 최신 입력을 유지합니다. */
    event_tail = (uint8_t)((event_tail + 1U) % IR_EVENT_QUEUE_DEPTH);
  }

  event_queue[event_head].key = key;
  event_queue[event_head].is_repeat = is_repeat ? 1U : 0U;
  event_queue[event_head].valid = 1U;
  event_head = next;
}

/**
 * @brief 디코딩된 키값을 이벤트 큐에 할당하는 콜백 함수입니다.
 */
void IR_Complete_Callback(uint8_t key) {
  if (key == 0xFFU) {
    return;
  }
  last_key = key;
  have_last_key = 1U;
  push_event(key, 0U);
}

/**
 * @brief 타이머 캡처 인터럽트에서 호출되어 리모컨 신호를 수신하고 즉시 처리합니다.
 */
void IR_Handle_Interrupt(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart) {
  (void)huart;

  if (htim == NULL || htim->Instance != TIM3) {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    uint32_t total_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    uint32_t high_duration = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

    /* 1. 리더 펄스 감지 (NEC 프로토콜 시작 신호: ~9ms + ~4.5ms) */
    if (total_period > IR_LEADER_PERIOD_US &&
        high_duration > IR_LEADER_HIGH_US) {
      ir_idx = 0U;
    }
    /* 1-b. NEC repeat (~9ms + ~2.25ms): 마지막 키를 deadman용으로 재발행 */
    else if (total_period > IR_LEADER_PERIOD_US &&
             high_duration >= IR_REPEAT_HIGH_MIN_US &&
             high_duration < IR_REPEAT_HIGH_MAX_US) {
      if (have_last_key != 0U) {
        push_event(last_key, 1U);
      }
      ir_idx = 0U;
    }
    /* 2. 데이터 비트 수집 */
    else if (ir_idx < 32U) {
      ir_raw_buf[ir_idx++] = total_period;

      /* 3. 32비트(1패킷) 수집 완료 시 즉시 처리 */
      if (ir_idx == 32U) {
        uint8_t key = IR_Decode_Packet(ir_raw_buf, 32U);
        IR_Complete_Callback(key); /* 콜백 호출 */
        ir_idx = 0U;               /* 다음 수신을 위해 초기화 */
      }
    }
  }
}

void IrRemote_Init(void) {
  ir_idx = 0U;
  event_head = 0U;
  event_tail = 0U;
  last_key = 0xFFU;
  have_last_key = 0U;
  last_activity_ms = 0U;
}

HAL_StatusTypeDef IrRemote_Start(void) {
  HAL_StatusTypeDef status;

  status = HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
  if (status != HAL_OK) {
    return status;
  }
  return HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
}

uint8_t IrRemote_PollEvent(IrRemoteEvent_t *event) {
  uint32_t primask;

  if (event == NULL) {
    return 0U;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (event_tail == event_head) {
    if (primask == 0U) {
      __enable_irq();
    }
    event->valid = 0U;
    return 0U;
  }

  *event = event_queue[event_tail];
  event_tail = (uint8_t)((event_tail + 1U) % IR_EVENT_QUEUE_DEPTH);
  if (primask == 0U) {
    __enable_irq();
  }
  return 1U;
}

uint32_t IrRemote_LastActivityMs(void) {
  return last_activity_ms;
}

void IrRemote_NoteActivity(uint32_t now_ms) {
  last_activity_ms = now_ms;
}
