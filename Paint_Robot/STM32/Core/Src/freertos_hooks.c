/**
 ******************************************************************************
 * @file    freertos_hooks.c
 * @brief   FreeRTOS 정적 idle task 메모리와 치명 오류 hook
 ******************************************************************************
 */

#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/** @brief FreeRTOS idle task가 사용할 정적 TCB입니다. */
static StaticTask_t idle_task_tcb;

/** @brief FreeRTOS idle task가 사용할 정적 stack입니다. */
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

/**
 * @brief kernel이 idle task를 만들 때 사용할 정적 메모리를 제공합니다.
 * @param[out] tcb_buffer idle task TCB 주소입니다.
 * @param[out] stack_buffer idle task stack 주소입니다.
 * @param[out] stack_size stack 크기 [StackType_t]입니다.
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **tcb_buffer,
                                   StackType_t **stack_buffer,
                                   uint32_t *stack_size) {
  *tcb_buffer = &idle_task_tcb;
  *stack_buffer = idle_task_stack;
  *stack_size = configMINIMAL_STACK_SIZE;
}

/**
 * @brief task stack overflow를 즉시 fail-safe 처리합니다.
 * @param task overflow가 발생한 task handle입니다.
 * @param task_name overflow가 발생한 task 이름입니다.
 */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name) {
  (void)task;
  (void)task_name;
  Error_Handler();
}

/**
 * @brief configASSERT 실패를 치명 오류 처리기로 전달합니다.
 * @param file assert가 발생한 소스 파일입니다.
 * @param line assert가 발생한 소스 행입니다.
 */
void vApplicationAssert(const char *file, int line) {
  (void)file;
  (void)line;
  Error_Handler();
}
