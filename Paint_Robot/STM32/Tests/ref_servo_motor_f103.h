#ifndef INC_SERVO_MOTOR_H_
#define INC_SERVO_MOTOR_H_

#include "main.h"

// 서보 모터 각도 상수
#define SET_ZERO    0.0f
#define SET_CENTER  135.0f
#define SET_LIMIT   270.0f

// 전역 변수로 관리되던 배열을 외부에서 참조할 수 있게 선언 (필요 시)
extern float M1_Arr[5];
extern float M2_Arr[5];

void Servo_Set_Angle(TIM_HandleTypeDef *htim, uint32_t channel, float angle);

#endif /* INC_SERVO_MOTOR_H_ */
