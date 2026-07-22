#include "servo_motor.h"

// main.c에서 정의된 배열들을 정의 (또는 main.c에 두고 extern으로 사용)
float M1_Arr[5] = { 90.0, 110.0, 135.0, 150.0, 170.0 };
float M2_Arr[5] = { 0.0, 15.0, 45.0, 60.0, 90.0 };

/**
 * @brief  서보 모터의 각도를 설정합니다.
 * @param  htim: 타이머 핸들 (예: &htim2)
 * @param  channel: 타이머 채널 (예: TIM_CHANNEL_1)
 * @param  angle: 설정할 각도 (0.0 ~ 270.0)
 */
void Servo_Set_Angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle)
{
    if (angle < 0.0f)   angle = 0.0f;
    if (angle > 270.0f) angle = 270.0f;

    // 270도 서보 모터 기준 (500us ~ 2500us)
    // 0도 -> 500us, 270도 -> 2500us
    uint32_t pulse_width = (uint32_t)(500.0f + (angle * 2000.0f / 270.0f));

    __HAL_TIM_SET_COMPARE(htim, channel, pulse_width);
}
