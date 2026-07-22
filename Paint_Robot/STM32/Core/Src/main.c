/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "usart.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "app_rtos.h"
#include "motor.h"
#include "servo.h"
#include "task.h"
#include "uart_transport.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static HAL_StatusTypeDef Robot_Init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
 * @brief 모터, 노즐 및 정적 RTOS application을 안전한 순서로 초기화합니다.
 * @return 모든 모듈이 시작되면 HAL_OK, 하나라도 실패하면 해당 오류입니다.
 */
static HAL_StatusTypeDef Robot_Init(void) {
  HAL_StatusTypeDef status = Servo_Init();
  if (status != HAL_OK) {
    return status;
  }

  status = Motor_Init();
  if (status != HAL_OK) {
    return status;
  }

  status = AppRtos_Init(&huart1);
  if (status != HAL_OK) {
    return status;
  }

  static uint8_t boot_msg[] =
      "=== STM32 Paint Robot Binary Control Ready ===\r\n";
  (void)HAL_UART_Transmit(&huart2, boot_msg, sizeof(boot_msg) - 1U, 100U);
  return HAL_OK;
}
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  /*
   * 초기화 순서가 중요합니다.
   * 1) GPIO가 먼저 EN=HIGH(드라이버 비활성) 안전 상태를 만듭니다.
   * 2) TIM1 서보 PWM을 OFF 위치에서 시작합니다.
   * 3) TIM2 20 kHz 모터 실시간 interrupt를 시작합니다.
   * 4) TIM3 IR PWM-input을 시작합니다.
   * 5) 정적 task/queue를 만든 뒤 scheduler에서 USART1 수신을 시작합니다.
   */
  /*
   * USART1(PA9 TX/PA10 RX): V-[HW] UART 바이너리 프레임, 115200-8-N-1
   * USART2(PA2 TX/PA3 RX): ST-Link Virtual COM 디버그 출력, 115200-8-N-1
   * TIM3_CH1(PB4): IR 수신기
   */
  if (Robot_Init() != HAL_OK) {
    Error_Handler();
  }
  vTaskStartScheduler();
  Error_Handler();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief HAL UART 수신 완료 callback을 UART transport로 전달합니다.
 * @param huart 수신 완료 이벤트를 발생시킨 UART 핸들입니다.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  UartTransport_RxCpltCallback(huart);
}

/**
 * @brief HAL UART 송신 완료 callback을 UART transport로 전달합니다.
 * @param huart 송신 완료 이벤트를 발생시킨 UART 핸들입니다.
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  UartTransport_TxCpltCallback(huart);
}

/**
 * @brief HAL UART 오류 callback을 UART transport 복구 처리로 전달합니다.
 * @param huart 오류 이벤트를 발생시킨 UART 핸들입니다.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  UartTransport_ErrorCallback(huart);
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /*
   * Fail-safe: GPIO 초기화 이후 발생한 치명 오류에서는 STEP을 LOW로 만들고
   * 두 DRV8825를 즉시 비활성화(EN=HIGH)한 뒤 정지합니다.
   */
  __disable_irq();
  if (__HAL_RCC_GPIOB_IS_CLK_ENABLED()) {
    Motor_ForceDisable();
  }
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
