```bash
Paint_Robot/STM32/Core/
├── Inc/ & Src/
│   ├── [구동/액추에이터 드라이버]
│   │   ├── motor.c / .h         ➔ 20 kHz TIM2 펄스 생성 & Bresenham 선형 가감속 엔진
│   │   └── servo.c / .h         ➔ TIM1_CH1 50 Hz PWM 기반 노즐 리프팅 제어
│   ├── [통신 및 패킷 프로토콜]
│   │   ├── usart.c & uart_transport.c ➔ USART1 115200 bps Non-blocking 링 버퍼 수신 드라이버
│   │   └── uart_protocol.c / .h ➔ 바이너리 프레임 직렬화/역직렬화 및 체크섬 무결성 검증
│   ├── [사용자 입력 및 안전 중재]
│   │   ├── ir_remote.c / .h     ➔ TIM3_CH1 기반 NEC 적외선 리모컨 수신 디코더
│   │   ├── control_arbiter.c / .h ➔ ESTOP / IR 수동 / UART 자율주행 3단계 제어권 중재기
│   │   └── robot_control.c / .h ➔ 현재 스텝 및 상태 플래그 기반 텔레메트리 조립기
│   └── [시스템 및 RTOS 스케줄링]
│       ├── app_rtos.c / .h      ➔ FreeRTOS 태스크 생성(ControlTask/CommTask) 및 우선순위 큐 관리
│       └── freertos_hooks.c     ➔ 스택 오버플로우 감시 및 시스템 크래시 방지 Fail-Safe
```
