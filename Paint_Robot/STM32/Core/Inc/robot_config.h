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
 * PDF 이론 90°=2012는 1/16 기준입니다.
 * 현재 1/16: MODE0=LOW, MODE1=LOW, MODE2=HIGH.
 * 하드웨어를 1/32로 바꾸면 이 값도 32.0f로 바꾸고 이론 펄스를 다시 계산하십시오.
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
 * PDF 예시·이론 90°=2012와 맞는 값입니다 (D=66, μ=16).
 * 실차 재실측 시 W를 바꾸면 이론 펄스도 다시 계산하십시오.
 * pulses_90 ≈ (200 × μ × W) / (4 × D)
 */
#define ROBOT_WHEEL_TRACK_MM 166.0f

/* ==========================================================================
 * 2. 방향/드라이버 설정
 * ========================================================================== */

/**
 * DIR 핀이 아래 레벨일 때 각 바퀴가 차체 전진 방향으로 돌아야 합니다.
 * 첫 공중 구동시험에서 반대로 도는 쪽만 SET <-> RESET으로 변경하십시오.
 * 좌우 모터가 거울 대칭 장착되면 두 값이 서로 다를 가능성이 큽니다.
 */
#define ROBOT_LEFT_FORWARD_LEVEL  GPIO_PIN_SET
/** 좌우 모터가 거울 대칭 장착이면 한쪽만 반전합니다. 전진이 회전하면 이 값을 바꾸십시오. */
#define ROBOT_RIGHT_FORWARD_LEVEL GPIO_PIN_RESET

/** @brief DRV8825 nENBL 출력 활성 레벨(Active-Low). */
#define ROBOT_DRIVER_ENABLE_LEVEL GPIO_PIN_RESET

/** @brief DRV8825 nENBL 출력 차단 레벨. */
#define ROBOT_DRIVER_DISABLE_LEVEL GPIO_PIN_SET

/**
 * @brief 정상 정지 시 홀딩 토크 유지 여부입니다.
 * @note ESTOP 완료 시에는 이 값과 관계없이 출력을 차단합니다.
 */
#define ROBOT_HOLD_TORQUE_WHEN_IDLE 1U

/* ==========================================================================
 * 3. TIM2 실시간 스텝 제어
 * ========================================================================== */

/**
 * @brief TIM2 update interrupt 주파수 [Hz].
 * @details 20 kHz에서는 50 us tick이며 STEP HIGH를 최소 한 tick 유지합니다.
 */
#define ROBOT_MOTOR_TICK_HZ 20000U

/**
 * @brief 좌우 목표 속도의 최대 절댓값 [microsteps/s].
 * @note IR/UART SPS는 이 값을 넘지 못합니다. 턴을 빠르게 하려면 여기부터 올리십시오.
 *       TIM2 20 kHz 구조상 실용 상한은 약 10000입니다. 탈조 나면 내리십시오.
 */
#define ROBOT_MAX_SPS 4000

/**
 * @brief 정상 주행 가감속도 [microsteps/s^2].
 * @note IR 펄스는 정지 시 감속 거리(v^2/2a)만큼 일찍 브레이크합니다.
 */
#define ROBOT_ACCEL_SPS2       3000U

/** @brief ESTOP 감속에 적용할 가감속도 [microsteps/s^2]. */
#define ROBOT_ESTOP_DECEL_SPS2 4000U

/** @brief 유효 SET_SPEED 미수신 시 자체 ESTOP까지의 시간 [ms]. */
#define ROBOT_UART_WATCHDOG_MS 300U

/** @brief 0x81 STATUS 송신 주기 [ms], 기본값은 10 Hz입니다. */
#define ROBOT_STATUS_PERIOD_MS 100U

/** @brief CLEAR_ESTOP(0x04) 안전키이며 Little-Endian payload는 5A A5입니다. */
#define ROBOT_CLEAR_ESTOP_KEY 0xA55AU

/* --------------------------------------------------------------------------
 * IR 수동 조작 — 키 1회 = 아래 펄스만큼만 움직이고 정지
 * 수정 파일: 이 robot_config.h 만
 * -------------------------------------------------------------------------- */

/**
 * @brief 타임아웃 계산용 실제 속도 (= motor.c clamp와 동일).
 * @note SPS를 5000으로 넣어도 MAX_SPS=2000이면 모터는 2000.
 *       타임아웃도 2000 기준으로 잡아야 펄스가 중간에 안 끊깁니다.
 */
#define ROBOT_IR_EFFECTIVE_SPS(sps)                                        \
  (((sps) < (ROBOT_MAX_SPS)) ? (sps) : (ROBOT_MAX_SPS))

/** @brief 앞/뒤 속도 [microsteps/s]. */
#define ROBOT_MANUAL_DRIVE_SPS 3200

#define ROBOT_IR_FWD_STEPS 6400U
#define ROBOT_IR_REV_STEPS 6400U

#define ROBOT_IR_FWD_MS                                                    \
  (((ROBOT_IR_FWD_STEPS) * 1000U) /                                        \
       (uint32_t)ROBOT_IR_EFFECTIVE_SPS(ROBOT_MANUAL_DRIVE_SPS) + 1500U)
#define ROBOT_IR_REV_MS                                                    \
  (((ROBOT_IR_REV_STEPS) * 1000U) /                                        \
       (uint32_t)ROBOT_IR_EFFECTIVE_SPS(ROBOT_MANUAL_DRIVE_SPS) + 1500U)

/**
 * @brief 좌/우 턴 속도 [microsteps/s]. ★속도만 조절★
 * @note 각도(이론×K_slip)와 별개. 더 빠르게: 이 값과 MAX_SPS를 같이 올림.
 */
#define ROBOT_MANUAL_TURN_LEFT_SPS 4000
#define ROBOT_MANUAL_TURN_RIGHT_SPS 4000

/* --------------------------------------------------------------------------
 * 제자리 회전: 이론 펄스 × K_slip  (PDF kinematics)
 *
 * 공식 (no-slip):
 *   P_theory(90°) = (N × μ × W) / (4 × D)  → 2012
 *     N=200, μ=16, W=166 mm, D=66 mm
 *   P_cmd = round( P_theory × K_slip )
 *
 * 캘리브 (STM only):
 *  - 2012는 고정. ROBOT_TURN_K_SLIP_MILLI_* 만 ±10(0.01)씩 노가다로 90° 맞춤.
 *  - 90°가 맞으면 STEPS_FROM_* 로 N도 명령도 같은 스케일로 따라갑니다.
 *
 * ★평소에 건드릴 값: ROBOT_TURN_K_SLIP_MILLI_* 만★
 * -------------------------------------------------------------------------- */

/** @brief 캘리브/명령 기준 각도 [0.1°]. 90.0° = 900. */
#define ROBOT_TURN_CAL_DECI_DEG 900U

/**
 * @brief PDF no-slip 이론 90° 펄스. 기구(W/D/μ)가 바뀌기 전에는 고정.
 * @note 1/32로 바꾸면 약 4024. 점퍼·MICROSTEP과 반드시 일치시키십시오.
 */
#define ROBOT_TURN_PULSE_THEORY_90 2012U

/**
 * @brief ★노가다★ 슬립 보정 K_slip [milli]. 1000=1.00, 1010=1.01, 990=0.99.
 * @note IR로 90° 돌리고 덜/더 돌면 ±10씩 조정. 이론 2012는 건드리지 말 것.
 */
#define ROBOT_TURN_K_SLIP_MILLI_LEFT  2010U
#define ROBOT_TURN_K_SLIP_MILLI_RIGHT 2010U

/** @brief 90° 실명령 펄스 = theory × K / 1000 (반올림). */
#define ROBOT_TURN_CAL_PULSES_LEFT                                         \
  ((((ROBOT_TURN_PULSE_THEORY_90) * (ROBOT_TURN_K_SLIP_MILLI_LEFT)) +      \
    500U) /                                                                \
   1000U)
#define ROBOT_TURN_CAL_PULSES_RIGHT                                        \
  ((((ROBOT_TURN_PULSE_THEORY_90) * (ROBOT_TURN_K_SLIP_MILLI_RIGHT)) +     \
    500U) /                                                                \
   1000U)

/**
 * @brief 각도[0.1°] → 펄스 (반올림). N도 회전에 사용.
 * @param deci 예: 90°→900, 45°→450, 0.1°→1
 */
#define ROBOT_TURN_STEPS_FROM_DECI_DEG_LEFT(deci)                          \
  ((((((uint32_t)(deci)) * (ROBOT_TURN_PULSE_THEORY_90) *                  \
      (ROBOT_TURN_K_SLIP_MILLI_LEFT)) +                                    \
     ((ROBOT_TURN_CAL_DECI_DEG) * 500U)) /                                 \
    ((ROBOT_TURN_CAL_DECI_DEG) * 1000U)))

#define ROBOT_TURN_STEPS_FROM_DECI_DEG_RIGHT(deci)                         \
  ((((((uint32_t)(deci)) * (ROBOT_TURN_PULSE_THEORY_90) *                  \
      (ROBOT_TURN_K_SLIP_MILLI_RIGHT)) +                                   \
     ((ROBOT_TURN_CAL_DECI_DEG) * 500U)) /                                 \
    ((ROBOT_TURN_CAL_DECI_DEG) * 1000U)))

/** @brief 각도[정수 °] → 펄스. */
#define ROBOT_TURN_STEPS_FROM_DEG_LEFT(deg)                                \
  ROBOT_TURN_STEPS_FROM_DECI_DEG_LEFT(((uint32_t)(deg)) * 10U)
#define ROBOT_TURN_STEPS_FROM_DEG_RIGHT(deg)                               \
  ROBOT_TURN_STEPS_FROM_DECI_DEG_RIGHT(((uint32_t)(deg)) * 10U)

/** @brief 참고: 1° / 0.1° 당 펄스 (K 반영). */
#define ROBOT_TURN_PULSES_PER_DEG_LEFT  ROBOT_TURN_STEPS_FROM_DEG_LEFT(1U)
#define ROBOT_TURN_PULSES_PER_DEG_RIGHT ROBOT_TURN_STEPS_FROM_DEG_RIGHT(1U)
#define ROBOT_TURN_PULSES_PER_DECI_DEG_LEFT                                \
  ROBOT_TURN_STEPS_FROM_DECI_DEG_LEFT(1U)
#define ROBOT_TURN_PULSES_PER_DECI_DEG_RIGHT                               \
  ROBOT_TURN_STEPS_FROM_DECI_DEG_RIGHT(1U)

/** @brief IR 90° 턴 = theory × K. */
#define ROBOT_IR_TURN_90_STEPS_LEFT  (ROBOT_TURN_CAL_PULSES_LEFT)
#define ROBOT_IR_TURN_90_STEPS_RIGHT (ROBOT_TURN_CAL_PULSES_RIGHT)

#define ROBOT_IR_TURN_90_MS_LEFT                                           \
  (((ROBOT_IR_TURN_90_STEPS_LEFT) * 1000U) /                               \
       (uint32_t)ROBOT_IR_EFFECTIVE_SPS(ROBOT_MANUAL_TURN_LEFT_SPS) +      \
   1500U)
#define ROBOT_IR_TURN_90_MS_RIGHT                                          \
  (((ROBOT_IR_TURN_90_STEPS_RIGHT) * 1000U) /                              \
       (uint32_t)ROBOT_IR_EFFECTIVE_SPS(ROBOT_MANUAL_TURN_RIGHT_SPS) +     \
   1500U)

/* ==========================================================================
 * 4. UART transport 정적 버퍼
 * ========================================================================== */

/** @brief FreeRTOS RX stream buffer의 수신 용량 [byte]. */
#define ROBOT_UART_RX_RING_SIZE 256U

/** @brief 비동기 STATUS 송신 ring queue의 슬롯 수입니다. */
#define ROBOT_UART_TX_QUEUE_DEPTH 4U

/* ==========================================================================
 * 5. PA8 / TIM1_CH1 도장 서보
 * ========================================================================== */

/** @brief 서보 PWM 주기 [us], 20 ms는 50 Hz입니다. */
#define ROBOT_SERVO_PERIOD_US 20000U

/** @brief 노즐 OFF 위치의 PWM HIGH 폭 [us], 실기구 시험 후 보정합니다. */
#define ROBOT_SERVO_OFF_US 1000U

/** @brief 노즐 ON 위치의 PWM HIGH 폭 [us], 실기구 시험 후 보정합니다. */
#define ROBOT_SERVO_ON_US 2000U

#endif /* __ROBOT_CONFIG_H__ */
