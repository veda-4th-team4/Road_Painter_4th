# Road-Painter STM32F401RE Firmware (Hard Real-Time Core)

STM32F401RE MCU(ARM Cortex-M4 @ 84 MHz)에서 구동되는 로봇 모터/서보 구동 및 하드웨어 안전 제어 펌웨어입니다. 상위 라즈베리파이와 115200 bps UART 바이너리 프로토콜로 실시간 통신하며, **20 kHz 타이머 인터럽트(TIM2)**에서 직접 스텝 펄스를 생성하여 펄스 지터(Jitter) 0ns의 모터 구동을 전담합니다.

---

## 📂 펌웨어 구조 및 모듈 책임 분리

```text
Paint_Robot/STM32/Core/
├── Inc/ & Src/
│   ├── [구동 / 액추에이터 계층]
│   │   ├── motor.c / .h         ➔ 20 kHz TIM2 하드웨어 ISR 펄스 생성 & Bresenham 선형 가감속 엔진
│   │   └── servo.c / .h         ➔ TIM1_CH1 50 Hz PWM 기반 MG996R 노즐 리프팅 제어
│   ├── [통신 및 패킷 프로토콜]
│   │   ├── usart.c              ➔ USART1 인터럽트 및 링 버퍼 수신 드라이버
│   │   ├── uart_transport.c     ➔ 논블로킹 패킷 송수신 인터페이스
│   │   └── uart_protocol.c / .h ➔ 바이너리 프레임 직렬화/역직렬화 및 8-bit Sum 체크섬 검증
│   ├── [사용자 입력 및 안전 중재]
│   │   ├── ir_remote.c / .h     ➔ TIM3_CH1 기반 NEC 적외선 리모컨 수신 디코더
│   │   ├── control_arbiter.c    ➔ ESTOP > IR 수동 > UART 자율주행 3단계 제어권 중재 & 300ms 워치독
│   │   └── robot_control.c      ➔ 실시간 좌/우 누적 스텝수 및 상태 플래그 텔레메트리 조립
│   └── [시스템 및 RTOS 스케줄링]
│       ├── app_rtos.c / .h      ➔ FreeRTOS 태스크(ControlTask +3 / CommTask +2) 스케줄링
│       └── freertos_hooks.c     ➔ 스택 오버플로우 감시 및 Fail-Safe 훅
└── Robot_Painter.ioc            ➔ STM32CubeMX 핀맵 및 타이머 페리페럴 설정 파일
```

---

## ⏱️ 3계층 우선순위(Priority) 아키텍처

| 실행 계층 | 모듈 / 함수 | 우선순위 | 주기 / 주파수 | 핵심 역할 |
| :--- | :--- | :--- | :--- | :--- |
| **Hard IRQ** | `TIM2_IRQHandler` | **최상위 (Hard IRQ)** | **20 kHz (50 µs)** | FreeRTOS 우회, Bresenham 선형 가감속 및 좌/우 STEP 펄스 하드웨어 직접 토글 |
| **Peripheral IRQ** | `USART1_IRQHandler`| 중위 (IRQ) | Event-driven | RPi 바이트 단위 수신 인터럽트 및 링 버퍼 적재 |
| **Peripheral IRQ** | `TIM3_IRQHandler` | 하위 (IRQ) | Event-driven | 적외선 리모컨 에지 캡처 |
| **FreeRTOS 제어** | `ControlTask` | **High (+3)** | **10 ms** | 300 ms 통신 워치독 감시, 다중 제어권 중재, 목표 SPS 갱신, 서보 PWM |
| **FreeRTOS 통신** | `CommTask` | **Normal (+2)** | Event-driven | 링 버퍼 바이너리 프레임 파싱 & 체크섬 검증 (모터 간섭 원천 차단) |

---

## 📦 UART 바이너리 통신 프로토콜 (115200 bps, 8N1)

프레임 구조: `[0xAA (STX)] [LEN (1B)] [CMD (1B)] [PAYLOAD (N Bytes)] [CHECKSUM (1B)]`

* `CMD 0x01 (SET_SPEED)`: 좌/우 목표 속도 (`int16_t left_sps`, `int16_t right_sps`)
* `CMD 0x02 (CONTROL_NOZZLE)`: 노즐 승강 제어 (`uint8_t state`: 1=DOWN, 0=UP)
* `CMD 0x03 (ESTOP)` / `CMD 0x04 (CLEAR_ESTOP)`: 비상 정지 및 래치 해제
* `CMD 0x07 (SET_SERVO_CONFIG)`: 부팅 시 동적 서보 PWM 펄스폭 전달 (`uint16_t off_us`, `uint16_t on_us`)
* `CMD 0x00 (STATUS)`: STM32가 50ms마다 RPi로 회신하는 현재 스텝수 (`int32_t left_steps`, `int32_t right_steps`, `uint8_t flags`)

---

## 🛡️ 안전 인터락 (Fail-Safe)
1. **300ms UART 통신 워치독**: 라즈베리파이로부터 300ms 이상 통신이 두절될 경우 `ControlTask`가 모터를 즉시 감속 정지시킵니다.
2. **다중 제어권 중재**: 비상정지(1순위) > 적외선 리모컨 수동 조작(2순위) > 상위 자율주행(3순위) 순으로 권한을 엄격히 중재합니다.

---

## 🛠️ 빌드 및 플래싱 가이드
1. **STM32CubeIDE**에서 `Paint_Robot/STM32` 프로젝트 열기
2. `Project` ➔ `Build Project` (`Ctrl + B`)
3. ST-Link V2/V3를 Nucleo-F401RE 보드에 연결 후 `Run` (`Ctrl + F11`)으로 펌웨어 다운로드
