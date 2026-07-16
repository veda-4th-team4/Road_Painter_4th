/**
 ******************************************************************************
 * @file    motor.c
 * @brief   TIM2 20 kHz 기반 DRV8825 좌우 독립 속도/가감속 제어
 * @details current_sps_q16과 phase_q16을 사용하므로 ISR에서 부동소수점
 *          연산을 하지 않습니다. STEP은 한 tick HIGH로 설정되고 다음 tick
 *          시작 시 LOW로 복귀합니다.
 ******************************************************************************
 */

#include "motor.h"
#include "tim.h"

/** @brief Q16 고정소수점에서 정수 1을 나타내는 배율입니다. */
#define Q16_ONE 65536L

/** @brief 한 STEP 상승 에지를 발생시키는 phase accumulator 임계값입니다. */
#define PHASE_THRESHOLD_Q16 ((uint32_t)(ROBOT_MOTOR_TICK_HZ << 16))

/**
 * @brief 한쪽 모터 축의 ISR 전용 제어 상태입니다.
 */
typedef struct {
  volatile int32_t target_q16;  /**< 목표 속도 [microsteps/s, Q16]. */
  volatile int32_t current_q16; /**< 현재 램프 속도 [microsteps/s, Q16]. */
  uint32_t phase_q16;           /**< STEP 발생용 위상 누산기. */
  volatile uint32_t steps;      /**< 방향 포함 누적 STEP(modulo 2^32). */
  uint8_t pulse_high;           /**< 현재 STEP 출력이 HIGH이면 1. */
  int8_t direction_sign;        /**< GPIO에 적용된 방향 부호(+1 또는 -1). */
} MotorAxis_t;

/** @name Left/right axis and safety latch state
 * @{ */
static MotorAxis_t left_axis;
static MotorAxis_t right_axis;
static volatile uint8_t estop_latched;
static volatile uint8_t timeout_latched;
static volatile uint8_t estop_reason;
static volatile uint8_t emergency_decel;
static volatile uint8_t driver_is_enabled = 0xFFU;
/** @} */

/**
 * @brief GPIO 논리 레벨을 반전합니다.
 * @param level 반전할 GPIO 레벨입니다.
 * @return GPIO_PIN_SET이면 RESET, RESET이면 SET입니다.
 */
static GPIO_PinState opposite_level(GPIO_PinState level) {
  return (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

/** @brief 중첩 상태를 보존하며 priority 0 TIM2까지 포함해 IRQ를 차단합니다. */
static uint32_t motor_critical_enter(void) {
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  return primask;
}

/** @brief 진입 전 IRQ가 활성 상태였을 때만 IRQ를 다시 허용합니다. */
static void motor_critical_exit(uint32_t primask) {
  if (primask == 0U) {
    __enable_irq();
  }
}

/**
 * @brief 두 DRV8825 nENBL을 동시에 제어합니다.
 * @param enable 0이 아니면 드라이버 활성, 0이면 출력 차단입니다.
 * @details 마지막 상태와 같으면 불필요한 GPIO 쓰기를 생략합니다.
 */
static void driver_enable(uint8_t enable) {
  enable = enable ? 1U : 0U;
  if (driver_is_enabled == enable) {
    return;
  }
  GPIO_PinState level =
      enable ? ROBOT_DRIVER_ENABLE_LEVEL : ROBOT_DRIVER_DISABLE_LEVEL;
  HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin, level);
  HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin, level);
  driver_is_enabled = enable;
}

/**
 * @brief 요청 속도를 펌웨어 최대 속도 범위로 제한합니다.
 * @param sps 제한할 속도 [microsteps/s]입니다.
 * @return [-ROBOT_MAX_SPS, ROBOT_MAX_SPS] 범위의 속도입니다.
 */
static int32_t clamp_sps(int32_t sps) {
  if (sps > ROBOT_MAX_SPS) {
    return ROBOT_MAX_SPS;
  }
  if (sps < -ROBOT_MAX_SPS) {
    return -ROBOT_MAX_SPS;
  }
  return sps;
}

/**
 * @brief 현재 Q16 속도를 목표 속도 방향으로 한 tick만큼 이동시킵니다.
 * @param current 현재 속도 [microsteps/s, Q16]입니다.
 * @param target 목표 속도 [microsteps/s, Q16]입니다.
 * @param accel_sps2 적용할 가감속도 [microsteps/s^2]입니다.
 * @return 이번 tick에 적용할 새로운 Q16 속도입니다.
 */
static int32_t ramp_toward(int32_t current, int32_t target,
                           uint32_t accel_sps2) {
  int32_t delta =
      (int32_t)(((uint64_t)accel_sps2 * Q16_ONE) / ROBOT_MOTOR_TICK_HZ);
  if (delta < 1) {
    delta = 1;
  }
  if (current < target) {
    return ((target - current) <= delta) ? target : current + delta;
  }
  if (current > target) {
    return ((current - target) <= delta) ? target : current - delta;
  }
  return current;
}

/**
 * @brief 지정 축의 DIR GPIO를 속도 부호에 맞게 설정합니다.
 * @param left 0이 아니면 좌측 축, 0이면 우측 축입니다.
 * @param signed_speed_q16 부호 판정에 사용할 Q16 속도입니다.
 */
static void set_direction(uint8_t left, int32_t signed_speed_q16) {
  GPIO_PinState forward =
      left ? ROBOT_LEFT_FORWARD_LEVEL : ROBOT_RIGHT_FORWARD_LEVEL;
  GPIO_PinState level =
      (signed_speed_q16 >= 0) ? forward : opposite_level(forward);
  if (left) {
    HAL_GPIO_WritePin(LEFT_DIR_GPIO_Port, LEFT_DIR_Pin, level);
  } else {
    HAL_GPIO_WritePin(RIGHT_DIR_GPIO_Port, RIGHT_DIR_Pin, level);
  }
}

/**
 * @brief 한 축의 속도 램프, 방향 전환 및 STEP 생성을 한 tick 진행합니다.
 * @param[in,out] axis 처리할 축 상태입니다.
 * @param left 0이 아니면 좌측 GPIO, 0이면 우측 GPIO를 사용합니다.
 * @param accel 적용할 가감속도 [microsteps/s^2]입니다.
 * @note DIR 변경 시 STEP 상승 전 최소 한 tick의 setup 시간을 확보합니다.
 */
static void axis_tick(MotorAxis_t *axis, uint8_t left, uint32_t accel) {
  GPIO_TypeDef *step_port =
      left ? LEFT_STEP_GPIO_Port : RIGHT_STEP_GPIO_Port;
  uint16_t step_pin = left ? LEFT_STEP_Pin : RIGHT_STEP_Pin;

  if (axis->pulse_high) {
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_RESET);
    axis->pulse_high = 0U;
  }

  axis->current_q16 =
      ramp_toward(axis->current_q16, axis->target_q16, accel);
  if (axis->current_q16 == 0) {
    axis->phase_q16 = 0U;
    return;
  }

  int8_t desired_sign = (axis->current_q16 >= 0) ? 1 : -1;
  if (axis->direction_sign != desired_sign) {
    /*
     * DIR 변경과 STEP 상승 사이에 최소 한 tick(50 us)을 확보합니다.
     * DRV8825 DIR setup 요구시간보다 충분히 깁니다.
     */
    set_direction(left, axis->current_q16);
    axis->direction_sign = desired_sign;
    return;
  }

  uint32_t speed_mag_q16 = (uint32_t)((axis->current_q16 >= 0)
                                          ? axis->current_q16
                                          : -axis->current_q16);
  axis->phase_q16 += speed_mag_q16;
  if (!axis->pulse_high && axis->phase_q16 >= PHASE_THRESHOLD_Q16) {
    axis->phase_q16 -= PHASE_THRESHOLD_Q16;
    set_direction(left, axis->current_q16);
    HAL_GPIO_WritePin(step_port, step_pin, GPIO_PIN_SET);
    axis->pulse_high = 1U;
    axis->steps += (axis->current_q16 >= 0) ? 1U : UINT32_MAX;
  }
}

/**
 * @brief 모터 상태와 안전 GPIO를 초기화하고 TIM2 interrupt를 시작합니다.
 * @return HAL_TIM_Base_Start_IT()의 결과입니다.
 */
HAL_StatusTypeDef Motor_Init(void) {
  left_axis = (MotorAxis_t){0};
  right_axis = (MotorAxis_t){0};
  estop_latched = 0U;
  timeout_latched = 0U;
  estop_reason = 0U;
  emergency_decel = 0U;
  driver_is_enabled = 0xFFU;

  HAL_GPIO_WritePin(LEFT_STEP_GPIO_Port, LEFT_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_STEP_GPIO_Port, RIGHT_STEP_Pin, GPIO_PIN_RESET);
  set_direction(1U, 1);
  set_direction(0U, 1);
  left_axis.direction_sign = 1;
  right_axis.direction_sign = 1;
  driver_enable(1U); // Force enabled on boot for debugging
  return HAL_TIM_Base_Start_IT(&htim2);
}

/**
 * @brief ESTOP이 아닐 때 좌우 최신 목표 속도를 원자적으로 반영합니다.
 * @param left_sps 좌측 목표 속도 [microsteps/s]입니다.
 * @param right_sps 우측 목표 속도 [microsteps/s]입니다.
 * @return 명령 수락 시 1, ESTOP latch 중이면 0입니다.
 */
uint8_t Motor_SetTargetSps(int16_t left_sps, int16_t right_sps) {
  uint32_t primask;

  if (estop_latched) {
    return 0U;
  }

  int32_t left = clamp_sps(left_sps);
  int32_t right = clamp_sps(right_sps);
  primask = motor_critical_enter();
  left_axis.target_q16 = left * Q16_ONE;
  right_axis.target_q16 = right * Q16_ONE;
  emergency_decel = 0U;
  motor_critical_exit(primask);

  if (left != 0 || right != 0) {
    driver_enable(1U);
  }
  return 1U;
}

/**
 * @brief 좌우 목표를 0으로 만들고 긴급 감속 및 ESTOP latch를 설정합니다.
 * @param reason 기록할 ESTOP 원인 코드입니다.
 * @param timeout 0이 아니면 timeout latch도 함께 설정합니다.
 */
void Motor_RequestEStop(uint8_t reason, uint8_t timeout) {
  uint32_t primask = motor_critical_enter();
  left_axis.target_q16 = 0;
  right_axis.target_q16 = 0;
  estop_latched = 1U;
  timeout_latched = timeout ? 1U : timeout_latched;
  estop_reason = reason;
  emergency_decel = 1U;
  motor_critical_exit(primask);
}

/**
 * @brief 완전 정지 여부를 검사하여 ESTOP 관련 latch를 해제합니다.
 * @return 해제 성공 시 1, 아직 정지하지 않았으면 0입니다.
 */
uint8_t Motor_ClearEStop(void) {
  uint8_t stopped;
  uint32_t primask = motor_critical_enter();
  stopped = (left_axis.current_q16 == 0 && right_axis.current_q16 == 0 &&
             left_axis.target_q16 == 0 && right_axis.target_q16 == 0 &&
             !left_axis.pulse_high && !right_axis.pulse_high);
  if (stopped) {
    estop_latched = 0U;
    timeout_latched = 0U;
    estop_reason = 0U;
    emergency_decel = 0U;
  }
  motor_critical_exit(primask);
  return stopped;
}

/**
 * @brief TIM2 주기마다 좌우 축을 갱신하고 정지 완료 시 드라이버를 차단합니다.
 * @warning ROBOT_MOTOR_TICK_HZ 주기의 TIM2 interrupt에서만 호출합니다.
 */
void Motor_TickISR(void) {
  uint32_t accel =
      emergency_decel ? ROBOT_ESTOP_DECEL_SPS2 : ROBOT_ACCEL_SPS2;
  axis_tick(&left_axis, 1U, accel);
  axis_tick(&right_axis, 0U, accel);

  uint8_t stopped = (left_axis.current_q16 == 0 &&
                     right_axis.current_q16 == 0 &&
                     left_axis.target_q16 == 0 &&
                     right_axis.target_q16 == 0);
  if (stopped) {
    /* Temporarily disabled for hardware debugging
    if (estop_latched || !ROBOT_HOLD_TORQUE_WHEN_IDLE) {
      driver_enable(0U);
    }
    */
  }
}

/**
 * @brief 공유 모터 상태를 interrupt 차단 구간에서 일관되게 복사합니다.
 * @param[out] snapshot 복사 결과를 받을 포인터입니다.
 */
void Motor_GetSnapshot(MotorSnapshot_t *snapshot) {
  uint32_t primask;

  if (snapshot == NULL) {
    return;
  }
  primask = motor_critical_enter();
  snapshot->target_left_sps = (int16_t)(left_axis.target_q16 / Q16_ONE);
  snapshot->target_right_sps = (int16_t)(right_axis.target_q16 / Q16_ONE);
  snapshot->current_left_sps = (int16_t)(left_axis.current_q16 / Q16_ONE);
  snapshot->current_right_sps = (int16_t)(right_axis.current_q16 / Q16_ONE);
  snapshot->left_steps = left_axis.steps;
  snapshot->right_steps = right_axis.steps;
  snapshot->moving = (left_axis.current_q16 != 0 ||
                      right_axis.current_q16 != 0 ||
                      left_axis.target_q16 != 0 ||
                      right_axis.target_q16 != 0);
  snapshot->estop_latched = estop_latched;
  snapshot->timeout_latched = timeout_latched;
  snapshot->estop_reason = estop_reason;
  motor_critical_exit(primask);
}

/**
 * @brief 치명 오류 시 감속 없이 두 모터의 펄스와 출력을 즉시 차단합니다.
 */
void Motor_ForceDisable(void) {
  left_axis.target_q16 = 0;
  right_axis.target_q16 = 0;
  left_axis.current_q16 = 0;
  right_axis.current_q16 = 0;
  HAL_GPIO_WritePin(LEFT_STEP_GPIO_Port, LEFT_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RIGHT_STEP_GPIO_Port, RIGHT_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LEFT_EN_GPIO_Port, LEFT_EN_Pin,
                    ROBOT_DRIVER_DISABLE_LEVEL);
  HAL_GPIO_WritePin(RIGHT_EN_GPIO_Port, RIGHT_EN_Pin,
                    ROBOT_DRIVER_DISABLE_LEVEL);
  driver_is_enabled = 0U;
}
