/**
 ******************************************************************************
 * @file    servo.c
 * @brief   TIM1_CH1 compare 기반 도장 노즐 서보 제어 구현
 ******************************************************************************
 */

#include "servo.h"
#include "tim.h"

/** @brief STATUS에 보고할 현재 노즐 논리 상태입니다. */
static volatile uint8_t nozzle_on;

/** @brief 동적 노즐 OFF (UP) 및 ON (DOWN) PWM pulse width [us] 입니다. */
static uint16_t servo_off_us = ROBOT_SERVO_OFF_US;
static uint16_t servo_on_us  = ROBOT_SERVO_ON_US;

/**
 * @brief 노즐 OFF pulse width를 설정하고 PA8 PWM 출력을 시작합니다.
 * @return HAL_TIM_PWM_Start()의 결과입니다.
 */
HAL_StatusTypeDef Servo_Init(void) {
  nozzle_on = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, servo_off_us);
  return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @brief 요청 논리 상태에 맞춰 TIM1_CH1 compare 값을 변경합니다.
 * @param on 0이면 노즐 OFF, 0이 아니면 노즐 ON입니다.
 */
void Servo_SetNozzle(uint8_t on) {
  nozzle_on = on ? 1U : 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                        nozzle_on ? servo_on_us : servo_off_us);
}

/**
 * @brief 마지막으로 명령된 노즐 논리 상태를 반환합니다.
 * @return 노즐 ON이면 1, OFF이면 0입니다.
 */
uint8_t Servo_IsNozzleOn(void) {
  return nozzle_on;
}

/**
 * @brief RPi로부터 수신한 동적 서보 PWM pulse width 설정값을 갱신하고 현재 비교값을 즉시 갱신합니다.
 * @param off_us 노즐 OFF (UP) PWM HIGH 폭 [us]
 * @param on_us 노즐 ON (DOWN) PWM HIGH 폭 [us]
 */
void Servo_SetConfig(uint16_t off_us, uint16_t on_us) {
  /* 안전 클램핑 (500us ~ 2500us 표준 서보 범위 보장) */
  if (off_us < 500U) off_us = 500U;
  if (off_us > 2500U) off_us = 2500U;
  if (on_us < 500U) on_us = 500U;
  if (on_us > 2500U) on_us = 2500U;

  servo_off_us = off_us;
  servo_on_us  = on_us;

  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                        nozzle_on ? servo_on_us : servo_off_us);
}
