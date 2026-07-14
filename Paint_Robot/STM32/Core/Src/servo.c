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

/**
 * @brief 노즐 OFF pulse width를 설정하고 PA8 PWM 출력을 시작합니다.
 * @return HAL_TIM_PWM_Start()의 결과입니다.
 */
HAL_StatusTypeDef Servo_Init(void) {
  nozzle_on = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ROBOT_SERVO_OFF_US);
  return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @brief 요청 논리 상태에 맞춰 TIM1_CH1 compare 값을 변경합니다.
 * @param on 0이면 노즐 OFF, 0이 아니면 노즐 ON입니다.
 */
void Servo_SetNozzle(uint8_t on) {
  nozzle_on = on ? 1U : 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                        nozzle_on ? ROBOT_SERVO_ON_US
                                  : ROBOT_SERVO_OFF_US);
}

/**
 * @brief 마지막으로 명령된 노즐 논리 상태를 반환합니다.
 * @return 노즐 ON이면 1, OFF이면 0입니다.
 */
uint8_t Servo_IsNozzleOn(void) {
  return nozzle_on;
}
