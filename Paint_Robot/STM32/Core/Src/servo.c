#include "servo.h"
#include "tim.h"

static volatile uint8_t nozzle_on;

HAL_StatusTypeDef Servo_Init(void) {
  nozzle_on = 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ROBOT_SERVO_OFF_US);
  return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Servo_SetNozzle(uint8_t on) {
  nozzle_on = on ? 1U : 0U;
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,
                        nozzle_on ? ROBOT_SERVO_ON_US
                                  : ROBOT_SERVO_OFF_US);
}

uint8_t Servo_IsNozzleOn(void) {
  return nozzle_on;
}
