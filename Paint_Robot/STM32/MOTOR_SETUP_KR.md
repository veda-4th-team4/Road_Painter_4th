# V-Road-Painter STM32 최종 통신·배선·시험 가이드

## 1. 적용 기준과 구현 범위

`V-[HW] UART 통신-130726-071752.pdf`를 UART 통신의 정본으로 사용합니다.
프레임, 바이트 순서, `0x01 SET_SPEED`, CRC-8, 115200-8-N-1을 그대로
적용했습니다. PDF에 없는 안전 항목은 SRS 부록 C.5에 따라 확장했습니다.

- USART1 비동기 바이너리 RX/TX와 송신 큐
- `0x01 SET_SPEED`, `0x02 NOZZLE`, `0x03 ESTOP`
- SRS 안전 확장 `0x04 CLEAR_ESTOP`, STM32 송신 `0x81 STATUS`
- TIM2 20 kHz 좌우 독립 STEP과 Q16 정수 가감속
- 300 ms SET_SPEED watchdog과 감속 ESTOP latch
- PA8/TIM1_CH1 50 Hz 서보 PWM
- 10 Hz 누적 좌우 스텝/상태 보고

ASCII `straight_...`, `left_...`, `right_...`, `DONE/ERR` 경로는 제거했습니다.
RPi는 응답을 기다렸다 다음 명령을 보내는 방식이 아니라 최신 속도를 주기적으로
갱신하고, 별도로 들어오는 STATUS를 비동기로 처리해야 합니다.

## 2. UART 전기·포트 설정

- 속도: 115200 baud
- 데이터: 8 bit
- 패리티: 없음
- 정지 비트: 1
- 흐름 제어: 없음
- STM32 USART1 TX PA9 → Raspberry Pi RXD GPIO15
- STM32 USART1 RX PA10 ← Raspberry Pi TXD GPIO14
- STM32와 Raspberry Pi는 3.3 V UART입니다. 5 V UART를 직접 연결하지 않습니다.
- RPi, Nucleo, 두 DRV8825, 서보 전원 GND를 반드시 공통으로 연결합니다.
- PA10은 RPi 분리 시 가짜 수신을 막기 위해 내부 Pull-up을 사용합니다.
- USART2 PA2/PA3은 ST-Link Virtual COM 진단용이며 바이너리 제어에 쓰지 않습니다.

## 3. 바이너리 프레임

모든 프레임은 다음 순서입니다.

`AA | LEN | CMD | PAYLOAD[LEN] | CRC8 | 55`

- `STX = 0xAA`, `ETX = 0x55`
- `LEN`은 PAYLOAD 바이트 수만 의미합니다.
- 다중 바이트 정수는 Little-Endian입니다.
- CRC 범위는 `LEN`, `CMD`, `PAYLOAD`입니다. STX, CRC 자신, ETX는 제외합니다.
- CRC-8은 polynomial `0x07`, initial `0x00`, xor-out `0x00`,
  reflection 없음으로 계산합니다.
- CRC/길이/ETX가 잘못된 프레임은 실행하지 않고 폐기합니다.

### `0x01 SET_SPEED`: RPi → STM32

PAYLOAD 길이는 4바이트입니다.

- byte 0~1: `int16_t left_sps`
- byte 2~3: `int16_t right_sps`
- 단위: DRV8825 microsteps/s
- 양수: 전진, 음수: 후진, 0: 정지
- 펌웨어 제한: 각 축 `-2000`~`+2000` sps, 초과값은 제한값으로 포화

예: 좌우 `+500 sps`

`AA 04 01 F4 01 F4 01 B1 55`

RPi는 주행 중뿐 아니라 정지 명령도 100 ms 이하 주기로 계속 보내십시오.
유효 SET_SPEED가 300 ms 동안 끊기면 STM32가 노즐을 끄고 자체 ESTOP합니다.

### `0x02 NOZZLE`: RPi → STM32

PAYLOAD는 1바이트이며 `00=OFF`, `01=ON`입니다.

- ON 예: `AA 01 02 01 46 55`
- ESTOP latch 중 ON 요청은 실행하지 않습니다.
- ESTOP와 watchdog timeout은 노즐을 즉시 OFF로 만듭니다.

### `0x03 ESTOP`: RPi → STM32

PAYLOAD 1바이트는 원인 코드입니다. `00`은 기본 RPi 요청 원인으로 처리합니다.

- 예: `AA 01 03 01 53 55`
- 수신 즉시 목표 속도를 0으로 바꾸고 ESTOP 감속도를 적용합니다.
- 완전 정지 후 DRV8825 EN을 HIGH로 만들어 출력을 차단합니다.
- 새 SET_SPEED는 ESTOP latch를 자동 해제하지 않습니다.

### `0x04 CLEAR_ESTOP`: RPi → STM32

PAYLOAD는 안전키 `uint16_t 0xA55A`입니다. 전송 바이트는 `5A A5`입니다.

- 프레임: `AA 02 04 5A A5 7B 55`
- 좌우 축이 완전히 정지한 경우에만 latch를 해제합니다.
- 해제 전에 SET_SPEED 0 heartbeat를 시작하고, 해제 후에도 계속 전송하십시오.

### `0x81 STATUS`: STM32 → RPi

STM32가 100 ms마다 전송하며 PAYLOAD는 9바이트입니다.

- byte 0~3: `uint32_t left_steps`
- byte 4~7: `uint32_t right_steps`
- byte 8: flags

스텝 카운터는 전진 시 증가하고 후진 시 감소하는 32-bit modulo 값입니다.
RPi에서 방향 포함 누적값으로 사용할 때는 동일 비트 패턴을 `int32_t`로
재해석하십시오.

flags 비트:

- bit 0 `MOVING`
- bit 1 `ESTOP`
- bit 2 `TIMEOUT`
- bit 3 `NOZZLE_ON`
- bit 4 `RX_ERROR`
- bit 5 `TX_OVERFLOW`

개별 명령에 대한 블로킹 `DONE/ERR` 응답은 없습니다. 명령 수락과 실제 동작
상태는 연속 STATUS flags/step 값으로 판단합니다.

## 4. STM32 핀과 CubeMX 설정

모터/서보:

- PB0 `LEFT_STEP`: 좌측 DRV8825 STEP, GPIO output push-pull/high
- PB1 `LEFT_DIR`: 좌측 DRV8825 DIR, GPIO output push-pull/high
- PB6 `LEFT_EN`: 좌측 nENBL, LOW 활성, 부팅 초기값 HIGH
- PB2 `RIGHT_STEP`: 우측 DRV8825 STEP, GPIO output push-pull/high
- PB5 `RIGHT_DIR`: 우측 DRV8825 DIR, GPIO output push-pull/high
- PB7 `RIGHT_EN`: 우측 nENBL, LOW 활성, 부팅 초기값 HIGH
- PA8 `SIG_SERVO`: TIM1_CH1 PWM, 50 Hz

타이머:

- TIM1 clock 84 MHz, prescaler 83, period 19999, CH1 PWM
- TIM2 clock 84 MHz, prescaler 83, period 49, update IRQ 20 kHz
- TIM2 IRQ priority 0, USART1 priority 1, USART2 priority 2

통신:

- USART1 PA9/PA10, 115200-8-N-1, RX/TX, global interrupt
- USART2 PA2/PA3, 115200-8-N-1, ST-Link VCP 진단

`Robot_Painter.ioc`에 위 설정이 반영되어 있습니다. CubeMX 코드를 다시 생성한
뒤에는 TIM2 IRQ가 `Motor_TickISR()`를 직접 호출하는지, USART HAL callbacks가
`UartProtocol_*Callback()`으로 전달되는지 반드시 diff로 확인하십시오.

## 5. DRV8825와 전원 필수 확인

- CIH-4248은 1.8도/step, 정격 1.8 A/phase 모터입니다.
- MODE0=L, MODE1=L, MODE2=H일 때 코드 가정인 1/16 microstep입니다.
- nRESET과 nSLEEP이 확장보드에서 HIGH로 유지되어야 합니다.
- 각 DRV8825 VMOT-GND 바로 옆에 최소 100 uF 전해 커패시터를 둡니다.
- 전원 ON 상태에서 모터 커넥터를 탈착하지 않습니다.
- 코일 한 쌍은 A1/A2, 다른 한 쌍은 B1/B2에 연결합니다.
- 전류 제한은 실제 모듈 Rsense를 확인하고 제조사 공식으로 Vref를 설정합니다.
- 1.8 A 연속 운전은 방열판/강제 냉각이 필요할 수 있으므로 낮은 전류부터
  시작하고 모터와 드라이버 온도를 측정합니다.
- 서보 5 V 배선은 MCU 전원과 별도 가지로 분기하고 서보 근처에 벌크
  커패시터를 둡니다.
- Nucleo를 외부 5 V와 ST-Link USB로 동시에 공급하기 전에 보드 전원 점퍼와
  역급전 조건을 NUCLEO-F401RE 매뉴얼에서 확인합니다.
- 3S/4S 배터리 요구가 문서 간 다르므로 실제 배터리를 확정한 후 벅 입력 정격,
  커패시터 전압 정격, 저전압 차단 조건을 함께 확정해야 합니다.

## 6. 조정값

`Core/Inc/robot_config.h`에서 다음을 조정합니다.

- `ROBOT_LEFT_FORWARD_LEVEL`, `ROBOT_RIGHT_FORWARD_LEVEL`: 바퀴 전진 DIR 극성
- `ROBOT_MAX_SPS`: 최대 microsteps/s
- `ROBOT_ACCEL_SPS2`: 정상 가감속도
- `ROBOT_ESTOP_DECEL_SPS2`: ESTOP 감속도
- `ROBOT_SERVO_OFF_US`, `ROBOT_SERVO_ON_US`: 실제 링크 기구에 맞는 서보 위치
- `ROBOT_HOLD_TORQUE_WHEN_IDLE`: 정상 정지 시 홀딩 토크 유지 여부

거리/각도 경로 계획은 이제 RPi가 담당합니다. 바퀴 지름 66 mm, 200 full
steps/rev, 1/16 microstep, 직결 기준 이론상 약 15.43 microsteps/mm입니다.
실제 주행에서는 타이어 눌림, 미끄럼, 축간거리 오차가 있으므로 RPi의 odometry
상수를 실차 측정으로 보정하십시오.

## 7. 최초 시험 순서

1. 차체를 띄우고 모터 전원을 끈 상태에서 GND, TX/RX 교차, 단락을 확인합니다.
2. 서보 혼을 분리하고 NOZZLE OFF/ON PWM 방향과 기계 한계를 확인합니다.
3. SET_SPEED `+100,+100`을 100 ms 주기로 전송해 두 바퀴 전진을 확인합니다.
4. 반대로 도는 축의 `ROBOT_*_FORWARD_LEVEL`만 바꿉니다.
5. `+100,-100`, `-100,+100`으로 제자리 양방향 회전을 확인합니다.
6. 주행 중 SET_SPEED 송신을 끊어 300 ms timeout, 감속, 노즐 OFF, ESTOP/TIMEOUT
   flags를 확인합니다.
7. 잘못된 CRC 프레임이 모터/노즐 상태를 바꾸지 않고 RX_ERROR를 보고하는지
   확인합니다.
8. ESTOP 후 잘못된 키로 해제되지 않고, 완전 정지 뒤 `5A A5` 키로만
   해제되는지 확인합니다.
9. STATUS 좌우 누적 스텝의 부호/증가량과 실제 바퀴 방향을 대조합니다.
10. 저속에서 발열/탈조/전압강하를 확인한 뒤 단계적으로 제한 속도를 올립니다.

## 8. 실제 빌드 파일

- `Core/Inc/uart_protocol.h`, `Core/Src/uart_protocol.c`: 프레임/CRC/RX/TX/watchdog
- `Core/Inc/motor.h`, `Core/Src/motor.c`: Q16 가감속/STEP/ESTOP/step counter
- `Core/Inc/servo.h`, `Core/Src/servo.c`: NOZZLE PWM
- `Core/Inc/tim.h`, `Core/Src/tim.c`: TIM1/TIM2 설정
- `Core/Src/main.c`: 초기화, main service loop, HAL UART callback 연결
- `Core/Src/stm32f4xx_it.c`: TIM2와 USART IRQ
- `Core/Inc/robot_config.h`: 속도/안전/서보 조정값
- `Robot_Painter.ioc`: CubeMX 원본 설정
