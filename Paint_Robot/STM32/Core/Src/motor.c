#include "motor.h"
#include "tim.h"

/**
 ******************************************************************************
 * @file    motor.c
 * @brief   TIM2 20 kHz 기반 DRV8825 좌우 독립 속도/가감속 제어
 *
 * current_sps_q16과 phase_q16을 사용하므로 ISR에서 부동소수점 연산을 하지
 * 않습니다. STEP은 한 tick HIGH, 다음 tick LOW로 유지됩니다.
 ******************************************************************************
 */

#define Q16_ONE             65536L
#define PHASE_THRESHOLD_Q16 ((uint32_t)(ROBOT_MOTOR_TICK_HZ << 16))

typedef struct {
  volatile int32_t target_q16;
  volatile int32_t current_q16;
  uint32_t phase_q16;
  volatile uint32_t steps;
  uint8_t pulse_high;
  int8_t direction_sign;
} MotorAxis_t;

static MotorAxis_t left_axis;
static MotorAxis_t right_axis;
static volatile uint8_t estop_latched;
static volatile uint8_t timeout_latched;
static volatile uint8_t estop_reason;
static volatile uint8_t emergency_decel;
static volatile uint8_t driver_is_enabled = 0xFFU;

static GPIO_PinState opposite_level(GPIO_PinState level) {
  return (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

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

static int32_t clamp_sps(int32_t sps) {
  if (sps > ROBOT_MAX_SPS) {
    return ROBOT_MAX_SPS;
  }
  if (sps < -ROBOT_MAX_SPS) {
    return -ROBOT_MAX_SPS;
  }
  return sps;
}

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
  driver_enable(0U);
  return HAL_TIM_Base_Start_IT(&htim2);
}

uint8_t Motor_SetTargetSps(int16_t left_sps, int16_t right_sps) {
  if (estop_latched) {
    return 0U;
  }

  int32_t left = clamp_sps(left_sps);
  int32_t right = clamp_sps(right_sps);
  __disable_irq();
  left_axis.target_q16 = left * Q16_ONE;
  right_axis.target_q16 = right * Q16_ONE;
  emergency_decel = 0U;
  __enable_irq();

  if (left != 0 || right != 0) {
    driver_enable(1U);
  }
  return 1U;
}

void Motor_RequestEStop(uint8_t reason, uint8_t timeout) {
  __disable_irq();
  left_axis.target_q16 = 0;
  right_axis.target_q16 = 0;
  estop_latched = 1U;
  timeout_latched = timeout ? 1U : timeout_latched;
  estop_reason = reason;
  emergency_decel = 1U;
  __enable_irq();
}

uint8_t Motor_ClearEStop(void) {
  uint8_t stopped;
  __disable_irq();
  stopped = (left_axis.current_q16 == 0 && right_axis.current_q16 == 0 &&
             left_axis.target_q16 == 0 && right_axis.target_q16 == 0 &&
             !left_axis.pulse_high && !right_axis.pulse_high);
  if (stopped) {
    estop_latched = 0U;
    timeout_latched = 0U;
    estop_reason = 0U;
    emergency_decel = 0U;
  }
  __enable_irq();
  return stopped;
}

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
    if (estop_latched || !ROBOT_HOLD_TORQUE_WHEN_IDLE) {
      driver_enable(0U);
    }
  }
}

void Motor_GetSnapshot(MotorSnapshot_t *snapshot) {
  if (snapshot == NULL) {
    return;
  }
  __disable_irq();
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
  __enable_irq();
}

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
