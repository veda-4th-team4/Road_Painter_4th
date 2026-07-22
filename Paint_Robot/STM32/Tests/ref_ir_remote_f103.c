#include "ir_remote.h"
#include <stdio.h>

// 리모컨 수신을 위한 내부 변수 (캡슐화)
static uint32_t ir_raw_buf[32];
static uint8_t ir_idx = 0;

extern uint8_t remote_key;
extern volatile uint8_t remote_done_flag;

/**
 * @brief  RAW 펄스 버퍼의 데이터 구역을 분석하여 중복 연산 없이 키값을 반환합니다.
 */
uint8_t IR_Decode_Packet(volatile uint32_t* raw_buf, uint8_t length)
{
    // 최소 32개 비트 완결성 확인
    if (length < 32) return 0xFF;

    uint8_t key_byte = 0;

    /* *
     * 배열의 [16]번 부터 [23]번 까지가 실제 리모컨의 데이터 바이트 영역입니다.
     * 다른 비트는 검사할 필요도 없이, 딱 이 8개 방만 돌면서 8비트 변수에 밀어 넣습니다.
     */
    for (int i = 0; i < 8; i++)
    {
        if (raw_buf[16 + i] > IR_THRESHOLD)
        {
            key_byte |= (1 << (7 - i)); // 배열 순서대로 상위 비트부터 적재
        }
    }

    return key_byte;
}

/**
 * @brief  디코딩된 키값을 메인 루프 변수에 할당하는 콜백 함수입니다.
 */
void IR_Complete_Callback(uint8_t key)
{
    // 메인 루프를 위한 데이터 전달
    remote_key = key;
    remote_done_flag = 1;
}

/**
 * @brief  타이머 캡처 인터럽트에서 호출되어 리모컨 신호를 수신하고 즉시 처리합니다.
 */
void IR_Handle_Interrupt(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t total_period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t high_duration = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

        // 1. 리더 펄스 감지 (NEC 프로토콜 시작 신호)
        if (total_period > 8000 && high_duration > 4000)
        {
            ir_idx = 0;
        }
        // 2. 데이터 비트 수집
        else if (ir_idx < 32)
        {
            ir_raw_buf[ir_idx++] = total_period;

            // 3. 32비트(1패킷) 수집 완료 시 즉시 처리
            if (ir_idx == 32)
            {
                uint8_t key = IR_Decode_Packet(ir_raw_buf, 32);
                IR_Complete_Callback(key); // 콜백 호출
                ir_idx = 0; // 다음 수신을 위해 초기화
            }
        }
    }
}
