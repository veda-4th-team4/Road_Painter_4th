/**
 ******************************************************************************
 * @file    robot_config.h
 * @brief   V-Road-Painter 주행부 사용자 설정값 모음
 ******************************************************************************
 *
 * 이 파일은 사용자가 하드웨어 조립/실측 후 수정해야 하는 값을 한곳에 모읍니다.
 * main.c, gpio.c 같은 CubeMX 생성 파일에 기구 상수를 직접 넣지 마십시오.
 *
 * [확정 하드웨어]
 * - MCU       : NUCLEO-F401RE (STM32F401RETx, 84 MHz)
 * - 모터      : CIH-4248 NEMA17, 1.8 deg/step, 1.8 A/phase, 2상 4선, 2개
 * - 드라이버  : DRV8825 + CNI 확장보드, 2개
 * - 바퀴      : 외경 66 mm, 폭 26.5 mm, 12 mm 육각 내경
 * - 통신      : Raspberry Pi <-> USART1, 115200-8-N-1, 3.3 V UART
 *
 * [최종 펌웨어 범위]
 * - V-[HW] UART 통신 PDF의 바이너리 프레임 및 0x01 SET_SPEED를 정본으로 사용
 * - TIM2 20 kHz ISR 기반 좌우 독립 펄스, 가감속, ESTOP, STATUS, 서보 PWM
 *
 ******************************************************************************
 */

#ifndef __ROBOT_CONFIG_H__
#define __ROBOT_CONFIG_H__

#include "main.h"

/* ==========================================================================
 * 1. 반드시 확인하거나 실측해서 입력할 기구 상수
 * ========================================================================== */

/**
 * @brief 모터 1회전의 풀스텝 수.
 * CIH-4248 스텝각이 1.8도이므로 360 / 1.8 = 200. 이 값은 확정값입니다.
 */
#define ROBOT_MOTOR_FULL_STEPS_PER_REV 200.0f

/**
 * @brief DRV8825 마이크로스텝 분주값.
 *
 * 코드값과 확장보드 MODE0/MODE1/MODE2 점퍼가 반드시 같아야 합니다.
 * 현재 1/16 설정: MODE0=LOW, MODE1=LOW, MODE2=HIGH.
 * 하드웨어를 1/32로 바꾸면 이 값도 32.0f로 바꾸십시오.
 */
#define ROBOT_MICROSTEP_DIVISOR 16.0f

/**
 * @brief 모터축 회전수 / 바퀴축 회전수.
 *
 * 모터와 바퀴가 어댑터로 직결되어 같은 속도로 돌면 1.0f입니다.
 * 예: 모터가 2회전할 때 바퀴가 1회전하는 2:1 감속이면 2.0f입니다.
 * CAD 형상과 실제 조립에서 풀리/기어가 없는지 반드시 확인하십시오.
 */
#define ROBOT_GEAR_RATIO 1.0f

/**
 * @brief 실제 하중을 건 상태의 유효 바퀴 지름 [mm].
 * 현재 STM32는 sps를 그대로 실행하므로 이 값은 RPi odometry와 기구 기록용입니다.
 * 일정 누적 스텝을 저속 구동한 뒤 실제 이동거리로 유효 지름을 보정하십시오.
 */
#define ROBOT_WHEEL_DIAMETER_MM 66.0f

/**
 * @brief 좌우 구동 바퀴의 접촉 중심선 간 거리(track width) [mm].
 *
 * 현재 150 mm는 임시값이며 확정값이 아닙니다.
 * 차체 조립 후 좌측/우측 구동륜 중심면 사이를 실측하여 반드시 교체하십시오.
 * 현재 STM32가 각도 명령을 계산하지 않으므로 RPi 회전 kinematics의 기준값입니다.
 * 일정한 반대 방향 좌우 sps로 회전한 뒤 실제 회전각을 측정해 보정하십시오.
 *
 * 4개 옴니휠 구조라면 바닥과 실제로 하중을 공유하는 좌우 접촉 중심선을 기준으로
 * 측정하며, 미끄러짐 때문에 기하학적 CAD 치수와 유효값이 다를 수 있습니다.
 */
#define ROBOT_WHEEL_TRACK_MM 150.0f /* TODO(USER): 실차 실측값으로 교체 */

/* ==========================================================================
 * 2. 방향/드라이버 설정
 * ========================================================================== */

/**
 * DIR 핀이 아래 레벨일 때 각 바퀴가 차체 전진 방향으로 돌아야 합니다.
 * 첫 공중 구동시험에서 반대로 도는 쪽만 SET <-> RESET으로 변경하십시오.
 * 좌우 모터가 거울 대칭 장착되면 두 값이 서로 다를 가능성이 큽니다.
 */
#define ROBOT_LEFT_FORWARD_LEVEL  GPIO_PIN_SET
#define ROBOT_RIGHT_FORWARD_LEVEL GPIO_PIN_SET /* TODO(USER): 공중시험 확인 */

/* DRV8825 nENBL은 Active-Low: LOW=출력 활성, HIGH=출력 차단 */
#define ROBOT_DRIVER_ENABLE_LEVEL  GPIO_PIN_RESET
#define ROBOT_DRIVER_DISABLE_LEVEL GPIO_PIN_SET

/* 정지 시 홀딩 토크 유지. ESTOP 완료 시에는 이 값과 관계없이 출력을 차단합니다. */
#define ROBOT_HOLD_TORQUE_WHEN_IDLE 1U

/* ==========================================================================
 * 3. TIM2 실시간 스텝 제어
 * ========================================================================== */

/* TIM2 update IRQ 주파수. 50 us tick에서 STEP High/Low를 각각 한 tick 유지합니다. */
#define ROBOT_MOTOR_TICK_HZ 20000U

/* RPi가 보낼 수 있는 좌/우 목표 속도 절댓값 [microsteps/s] */
#define ROBOT_MAX_SPS 2000

/* 정상 및 ESTOP 감속도 [microsteps/s^2] */
#define ROBOT_ACCEL_SPS2       1200U
#define ROBOT_ESTOP_DECEL_SPS2 4000U

/* SET_SPEED가 이 시간 이상 끊기면 자체 ESTOP (SRS C.5) */
#define ROBOT_UART_WATCHDOG_MS 300U

/* 0x81 STATUS 송신 주기 = 10 Hz */
#define ROBOT_STATUS_PERIOD_MS 100U

/* CLEAR_ESTOP(0x04) 안전키. Little-Endian payload는 5A A5입니다. */
#define ROBOT_CLEAR_ESTOP_KEY 0xA55AU

/* ==========================================================================
 * 4. UART 바이너리 프로토콜 버퍼
 * ========================================================================== */

#define ROBOT_UART_RX_RING_SIZE 256U
#define ROBOT_UART_MAX_PAYLOAD   16U
#define ROBOT_UART_MAX_FRAME     (ROBOT_UART_MAX_PAYLOAD + 5U)
#define ROBOT_UART_TX_QUEUE_DEPTH 4U

/* ==========================================================================
 * 5. PA8 / TIM1_CH1 도장 서보
 * ========================================================================== */

#define ROBOT_SERVO_PERIOD_US 20000U /* 50 Hz */
#define ROBOT_SERVO_OFF_US     1000U /* 실기구 시험 후 보정 */
#define ROBOT_SERVO_ON_US      2000U /* 실기구 시험 후 보정 */

#endif /* __ROBOT_CONFIG_H__ */
