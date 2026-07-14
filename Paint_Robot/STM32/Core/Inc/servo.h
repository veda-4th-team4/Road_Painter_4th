/**
 ******************************************************************************
 * @file    servo.h
 * @brief   PA8/TIM1_CH1 도장 서보 제어
 * @details TIM1의 50 Hz PWM compare 값을 변경하여 노즐 ON/OFF 위치를
 *          제어하고, STATUS 생성에 사용할 논리 상태를 제공합니다.
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "robot_config.h"

/**
 * @brief 노즐을 OFF 위치로 설정하고 TIM1_CH1 PWM 출력을 시작합니다.
 * @return HAL_OK이면 시작 성공, 그 외에는 PWM 시작 실패입니다.
 * @pre MX_TIM1_Init()이 먼저 완료되어야 합니다.
 */
HAL_StatusTypeDef Servo_Init(void);

/**
 * @brief 노즐 서보의 논리 상태와 PWM pulse width를 갱신합니다.
 * @param on 0이면 ROBOT_SERVO_OFF_US, 0이 아니면 ROBOT_SERVO_ON_US입니다.
 * @warning ISR에서 호출하지 마십시오. ControlTask 전용 API입니다.
 */
void Servo_SetNozzle(uint8_t on);

/**
 * @brief 현재 명령된 노즐 논리 상태를 반환합니다.
 * @return 노즐 ON이면 1, OFF이면 0입니다.
 */
uint8_t Servo_IsNozzleOn(void);

#endif /* __SERVO_H__ */
