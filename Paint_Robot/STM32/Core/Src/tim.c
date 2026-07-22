/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    tim.c
 * @brief   TIM1 50 Hz servo PWM, TIM2 20 kHz motor tick, TIM3 IR capture
 * @details 84 MHz APB timer clock을 1 MHz counter clock으로 분주합니다.
 ******************************************************************************
 */
/* USER CODE END Header */

#include "tim.h"
#include "robot_config.h"

TIM_HandleTypeDef htim1; /**< 노즐 서보 PWM용 TIM1 HAL 핸들. */
TIM_HandleTypeDef htim2; /**< 모터 실시간 tick용 TIM2 HAL 핸들. */
TIM_HandleTypeDef htim3; /**< IR NEC 캡처용 TIM3 HAL 핸들. */

/**
 * @brief TIM1_CH1을 50 Hz PWM 모드로 초기화합니다.
 */
void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_OC_InitTypeDef pwm = {0};

  /* APB2 timer clock 84 MHz / (83+1) = 1 MHz, 20000 counts = 50 Hz */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = ROBOT_SERVO_PERIOD_US - 1U;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &clock) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &master) != HAL_OK) {
    Error_Handler();
  }
  pwm.OCMode = TIM_OCMODE_PWM1;
  pwm.Pulse = ROBOT_SERVO_OFF_US;
  pwm.OCPolarity = TIM_OCPOLARITY_HIGH;
  pwm.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  pwm.OCFastMode = TIM_OCFAST_DISABLE;
  pwm.OCIdleState = TIM_OCIDLESTATE_RESET;
  pwm.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &pwm, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

/**
 * @brief TIM2를 모터 제어용 ROBOT_MOTOR_TICK_HZ base timer로 초기화합니다.
 */
void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef clock = {0};
  TIM_MasterConfigTypeDef master = {0};

  /* APB1 timer clock 84 MHz / 84 = 1 MHz / 50 = 20 kHz */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 83;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (1000000U / ROBOT_MOTOR_TICK_HZ) - 1U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
    Error_Handler();
  }
  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &clock) != HAL_OK) {
    Error_Handler();
  }
  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &master) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief TIM3를 IR 수신기 PWM-input 모드로 초기화합니다.
 * @details PB4 TIM3_CH1, 1 us tick, CH1 falling / CH2 rising indirect.
 */
void MX_TIM3_Init(void) {
  TIM_ClockConfigTypeDef clock = {0};
  TIM_SlaveConfigTypeDef slave = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_IC_InitTypeDef config_ic = {0};

  /* APB1 timer clock 84 MHz / (83+1) = 1 MHz → 1 us */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 83;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  clock.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &clock) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  slave.SlaveMode = TIM_SLAVEMODE_RESET;
  slave.InputTrigger = TIM_TS_TI1FP1;
  slave.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  slave.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &slave) != HAL_OK) {
    Error_Handler();
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &master) != HAL_OK) {
    Error_Handler();
  }

  config_ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  config_ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
  config_ic.ICPrescaler = TIM_ICPSC_DIV1;
  config_ic.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &config_ic, TIM_CHANNEL_1) != HAL_OK) {
    Error_Handler();
  }
  config_ic.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  config_ic.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &config_ic, TIM_CHANNEL_2) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_ENABLE();
  } else if (htim_base->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    /* 20 kHz STEP timing이 UART보다 높은 우선순위를 가져야 합니다. */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  } else if (htim_base->Instance == TIM3) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB4 ------> TIM3_CH1 IR receiver input */
    GPIO_InitStruct.Pin = IR_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(IR_RX_GPIO_Port, &GPIO_InitStruct);

    /* TIM2(0)보다 낮고 FreeRTOS FromISR 허용 범위(>=5) 안입니다. */
    HAL_NVIC_SetPriority(TIM3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM1) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA8 -> TIM1_CH1 -> 회로 J_STM_SIG의 SIG_SERVO */
    GPIO_InitStruct.Pin = SIG_SERVO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(SIG_SERVO_GPIO_Port, &GPIO_InitStruct);
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef *htim_base) {
  if (htim_base->Instance == TIM1) {
    __HAL_RCC_TIM1_CLK_DISABLE();
  } else if (htim_base->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_DISABLE();
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  } else if (htim_base->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_DISABLE();
    HAL_GPIO_DeInit(IR_RX_GPIO_Port, IR_RX_Pin);
    HAL_NVIC_DisableIRQ(TIM3_IRQn);
  }
}
