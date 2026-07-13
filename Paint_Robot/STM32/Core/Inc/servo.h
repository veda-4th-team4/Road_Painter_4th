/**
 ******************************************************************************
 * @file    servo.h
 * @brief   PA8/TIM1_CH1 도장 서보 제어
 ******************************************************************************
 */

#ifndef __SERVO_H__
#define __SERVO_H__

#include "robot_config.h"

HAL_StatusTypeDef Servo_Init(void);
void Servo_SetNozzle(uint8_t on);
uint8_t Servo_IsNozzleOn(void);

#endif /* __SERVO_H__ */
