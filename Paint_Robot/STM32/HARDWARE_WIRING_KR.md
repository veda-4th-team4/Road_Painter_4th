# V-Road-Painter 하드웨어 배선 가이드 (최종)

이 문서는 **현재 펌웨어 핀맵**을 정본으로 합니다. 근거 파일은
`Robot_Painter.ioc`, `main.h`, `gpio.c`, `tim.c`, `usart.c`,
`MOTOR_SETUP_KR.md`입니다.

구성은 한 가지입니다.

- MCU: NUCLEO-F401RE
- 상위: Raspberry Pi (USART1 바이너리 제어)
- 구동: NEMA17 2개 + DRV8825 2개 (+ CNI 확장보드)
- 노즐: PA8 TIM1 서보 PWM
- 수동/비상: NEC IR 수신기 (PB4 / TIM3). **물리 ESTOP 버튼은 사용하지
  않습니다.** 비상 정지는 리모컨 POWER(원격 ESTOP)와 서버 `0x03 ESTOP`으로
  처리합니다.

실험실에서 USB·벤치 전원으로 조립하든, 배터리까지 갖춘 차체로 조립하든
**GPIO·UART·IR 핀은 동일**합니다. 달라지는 것은 전원 rail뿐입니다.

확장보드마다 실크 인쇄가 조금씩 다를 수 있습니다. `STEP/STP`,
`EN/ENABLE/nENBL`, `M0/MS1`처럼 이름만 다른 경우가 있으므로 반드시 실제
보드의 인쇄와 회로를 확인한 뒤 연결하십시오.

## 마이크로스텝 설정과 펌웨어 가정

확장 보드에 점퍼를 어떻게 꽂느냐에 따라 1.8도짜리 모터가 다음과 같이
쪼개져서 움직입니다. (표기는 A4988 계열과 동일한 MODE 조합 설명입니다.
DRV8825에서도 MODE0/1/2로 동일하게 설정합니다.)

| MODE2 MODE1 MODE0 (또는 점퍼) | 스텝 | 각도 | 1회전 microstep |
|---|---|---|---|
| 0 0 0 (점퍼 없음) | Full | 1.8° | 200 |
| 1 0 0 (1번만) | 1/2 | 0.9° | 400 |
| 0 1 0 (2번만) | 1/4 | 0.45° | 800 |
| 1 1 0 (1,2번) | 1/8 | 0.225° | 1600 |
| 1 1 1 (전부) | 1/16 | 0.1125° | 3200 |

**현재 펌웨어 `ROBOT_MICROSTEP_DIVISOR = 16`이므로 보드도 1/16이어야
합니다.** DRV8825에서는 `MODE0=LOW`, `MODE1=LOW`, `MODE2=HIGH`입니다.
점퍼가 HIGH에 붙는 보드라면 M2만 설치합니다. DIP라면 M2만 ON입니다.
전원이 꺼진 상태에서 continuity로 실제로 HIGH에 붙는지 확인하십시오.

## 1. 가장 중요한 안전 규칙

1. 모터 전원이 켜진 상태에서 모터 또는 DRV8825를 꽂거나 빼지 않습니다.
2. DRV8825 방향을 반대로 꽂으면 Nucleo, 드라이버, Raspberry Pi가 파손될 수
   있습니다. `VMOT`, `GND`, 가변저항 위치를 먼저 확인합니다.
3. Nucleo의 `5V`, `3V3` 핀으로 NEMA17 모터에 전원을 공급하지 않습니다.
4. Raspberry Pi의 `5V` 또는 `3V3` 핀으로 모터에 전원을 공급하지 않습니다.
5. DRV8825의 모터 전원은 별도 전원장치 또는 배터리 motor rail에서
   공급합니다.
6. RPi, Nucleo, 좌우 DRV8825, 서보, IR 모듈 전원의 GND는 반드시
   공통으로 연결합니다.
7. VMOT 바로 옆에 드라이버마다 최소 100 uF 전해 커패시터를 설치합니다.
8. 첫 시험은 바퀴를 바닥에서 띄우고 낮은 전류와 `100 sps` 이하로
   진행합니다.
9. UART `0x03 ESTOP`과 리모컨 POWER는 소프트웨어 정지입니다. DRV8825
   `EN=HIGH`는 제어 신호 차단이며, 필요하면 전원 쪽 퓨즈/차단을 별도로
   고려하십시오.
10. IR 수신기는 3.3 V 논리만 사용합니다. 5 V OUT을 STM32에 직접 넣지
    않습니다.

## 2. NUCLEO-F401RE 최종 핀맵

아래 핀은 `Robot_Painter.ioc`, `main.h`, `gpio.c`, `tim.c`, `usart.c`와
일치합니다.

| 용도 | STM32 GPIO | Nucleo에서 찾기 쉬운 이름 | ST Morpho | 연결 대상 |
|---|---:|---|---|---|
| 좌측 STEP | PB0 | Arduino `A3` | `CN7-34` | 좌측 보드 `STEP` |
| 좌측 DIR | PB1 | Arduino 헤더 없음 | `CN10-24` | 좌측 보드 `DIR` |
| 좌측 ENABLE | PB6 | Arduino `D10` | `CN10-17` | 좌측 보드 `EN/nENBL` |
| 우측 STEP | PB2 | Arduino 헤더 없음 | `CN10-22` | 우측 보드 `STEP` |
| 우측 DIR | PB5 | Arduino `D4` | `CN10-29` | 우측 보드 `DIR` |
| 우측 ENABLE | PB7 | Arduino 헤더 없음 | `CN7-21` | 우측 보드 `EN/nENBL` |
| 노즐 서보 신호 | PA8 | Arduino `D7` | `CN10-23` | 서보 Signal |
| STM32 USART1 TX | PA9 | Arduino `D8` | `CN10-21` | RPi RXD GPIO15 |
| STM32 USART1 RX | PA10 | Arduino `D2` | `CN10-33` | RPi TXD GPIO14 |
| IR 수신기 OUT | PB4 | Arduino `D5` | `CN10-27` | IR module OUT (`IR_RX`) |
| USART2 TX (진단) | PA2 | ST-LINK VCP | — | PC Virtual COM |
| USART2 RX (진단) | PA3 | ST-LINK VCP | — | PC Virtual COM |

PC13(Nucleo B1)은 펌웨어에서 사용하지 않습니다. ESTOP 입력으로 연결하지
마십시오.

전원 및 GND:

- Nucleo `3V3`: `CN7-16` (IR VCC, RESET/SLEEP pull-up 후보)
- Nucleo `5V`: `CN7-18` (모터·서보 전원으로 쓰지 않음)
- Nucleo GND: Arduino GND 또는 Morpho `CN7-8/19/20/22`, `CN10-9/20`

Morpho 번호는 보드를 위에서 보고 USB 방향을 확인한 뒤 UM1724의 `CN7`/`CN10`
과 대조하십시오. GPIO 이름 없이 “몇 번째 핀”만 세면 방향에 따라 반대로
셀 수 있습니다.

### 2.1 타이머·IRQ 할당

| 자원 | 용도 | 비고 |
|---|---|---|
| TIM1 CH1 / PA8 | 서보 50 Hz PWM | |
| TIM2 | 모터 STEP 20 kHz | IRQ priority **0** |
| TIM3 CH1 / PB4 | IR PWM-input 1 µs | IRQ priority **6** |
| TIM5 | HAL 1 ms timebase | F401에 TIM6 없음, IRQ **15** |
| SysTick | FreeRTOS 1 kHz | priority **15** |
| USART1 | RPi 바이너리 | IRQ priority **6** |
| USART2 | 진단 / IR 로그 | IRQ priority **7** |

## 3. 모터·드라이버 배선

### 3.1 필요 부품

- 모터용 DC 전원 (첫 시험: 전류 제한 가능한 12 V, 3~5 A급에서 낮은
  제한부터)
- 또는 정식: 배터리 + 퓨즈 + motor rail
- DRV8825별 100~470 uF 전해 커패시터
  - 12 V → 25 V 이상 정격, 24 V → 35 V 이상 정격
- 점퍼선, 굵은 모터 전원선, 퓨즈 또는 재설정형 차단기
- 멀티미터, 가능하면 로직 애널라이저/오실로스코프
- DRV8825 방열판 (필요 시 팬)
- NEC IR 수신 모듈 (3.3 V)

### 3.2 좌측 DRV8825

| Nucleo | 좌측 확장보드 | 설명 |
|---|---|---|
| PB0 / A3 / CN7-34 | `STEP` / `STP` | 상승 에지마다 1 microstep |
| PB1 / CN10-24 | `DIR` | 회전 방향 |
| PB6 / D10 / CN10-17 | `EN` / `nENBL` | LOW 활성, HIGH 비활성 |
| GND | Logic GND | 신호 기준 |

부팅 시 펌웨어는 `EN=HIGH`입니다. 유효한 이동 명령이 들어오면 LOW로
바뀝니다. ESTOP·완전 정지 후에는 다시 HIGH로 차단합니다.

### 3.3 우측 DRV8825

| Nucleo | 우측 확장보드 | 설명 |
|---|---|---|
| PB2 / CN10-22 | `STEP` / `STP` | 상승 에지마다 1 microstep |
| PB5 / D4 / CN10-29 | `DIR` | 회전 방향 |
| PB7 / CN7-21 | `EN` / `nENBL` | LOW 활성, HIGH 비활성 |
| GND | Logic GND | 신호 기준 |

### 3.4 DRV8825 공통 설정

| DRV8825 단자 | 연결 |
|---|---|
| `nRESET` / `RST` | HIGH 유지 |
| `nSLEEP` / `SLP` | HIGH 유지 |
| `MODE0` / `M0` | LOW |
| `MODE1` / `M1` | LOW |
| `MODE2` / `M2` | HIGH (1/16) |
| `FAULT` | 현재 미사용 (향후 확장) |

`RESET`과 `SLEEP`은 둘 다 HIGH여야 동작합니다. 확장보드가 pull-up/jumper로
이미 HIGH면 추가 배선이 필요 없습니다. 직접 연결한다면 묶어 Nucleo `3V3`에
연결할 수 있습니다. 5 V로 HIGH를 만들더라도 그 전압을 STM32 GPIO로
되돌려 연결하면 안 됩니다.

### 3.5 모터 전원

| 전원 | DRV8825 |
|---|---|
| DC `+` / Motor rail `+` | `VMOT` / `M+` / `V+` |
| DC `-` / Motor rail `-` | `GND` / `M-` / `V-` |

커패시터: `+`→VMOT, `-`→모터 GND, 단자 바로 옆.

좌우를 같은 전원에서 쓸 때는 분배점에서 **각각 별도 가지**로 분기합니다.
한 드라이버를 거쳐 다른 쪽으로 직렬처럼 이어 붙이지 마십시오.

모터 전원 `+`는 Nucleo·RPi에 직접 연결하지 않습니다. 공통은 GND입니다.

### 3.6 NEMA17 코일 (CIH-4248, 2상 4선)

1. 모터를 전원과 완전히 분리합니다.
2. 멀티미터로 서로 통하는 두 선을 찾아 한 코일 쌍으로 둡니다.
3. 첫 코일 → `A1/A2` (또는 `1A/1B`).
4. 나머지 코일 → `B1/B2` (또는 `2A/2B`).

코일을 섞으면 떨기만 하고 돌지 않습니다.

방향이 반대면:

- 권장: `robot_config.h`의 `ROBOT_*_FORWARD_LEVEL` 변경
- 또는 전원 OFF 후 한 코일 쌍의 두 선만 교환

### 3.7 전류 제한

정격 1.8 A/phase라고 처음부터 1.8 A로 두지 않습니다. clone마다 Rsense와
Vref 식이 다릅니다.

1. 보드의 `R050`/`R100`/`R200` 표기를 확인합니다.
2. 판매처 Vref 식을 확인합니다.
3. 약 0.5 A부터 시작합니다.
4. 탈조·토크·온도를 보며 단계적으로 올립니다.
5. 1.8 A 근처는 방열판·강제 냉각이 필요할 수 있습니다.

Vref 측정 시 주변 핀 단락에 주의하십시오.

## 4. Raspberry Pi UART

| Raspberry Pi | Nucleo | 방향 |
|---|---|---|
| 물리 8번, GPIO14 TXD | PA10 / D2 / CN10-33 | RPi → STM32 |
| 물리 10번, GPIO15 RXD | PA9 / D8 / CN10-21 | STM32 → RPi |
| 물리 6번 GND | Nucleo GND | 공통 기준 |

- TX/RX는 교차입니다.
- 양쪽 모두 3.3 V UART → 레벨시프터 불필요.
- UART 배선에 RPi 5 V를 넣지 않습니다.
- serial console 비활성, hardware UART 활성.
- 장치 노드는 `/dev/ttyAMA0`, 115200-8-N-1이 GPIO14/15에 연결되는지
  확인합니다.
- 바이너리 프레임·명령은 `MOTOR_SETUP_KR.md`를 정본으로 합니다.
  (`0x01`~`0x05`, `0x81 STATUS`)

실험실에서는 RPi USB-C 전원 + Nucleo ST-LINK USB + GND 공통이 안전합니다.

## 5. IR 리모컨 배선

| IR 모듈 | Nucleo | 설명 |
|---|---|---|
| VCC | `3V3` (`CN7-16`) | 5 V 금지 |
| GND | 공통 GND | |
| OUT | PB4 / D5 / CN10-27 | TIM3_CH1, 내부 pull-up |

- NEC 프로토콜 가정. 키 raw 값은 USART2에 `[IR Key]: 0xXX`로 출력됩니다.
- 매크로는 `Core/Inc/ir_remote.h`의 `IR_KEY_*`를 실측값으로 교체합니다.
- 방향키는 hold 중 NEC repeat가 약 **200 ms** 이내로 들어와야 계속
  움직입니다 (`ROBOT_IR_DEADMAN_MS`).
- **AUTO(기본)** 에서는 방향·노즐 키는 거부되고 POWER(원격 ESTOP)만
  수락합니다.
- AUTO/MANUAL 모드 구분 없음. IR과 서버 SET_SPEED 모두 사용 가능(서버 주행 중에는 리모컨을 누르지 말 것).

| 매크로 | 기본값 | 동작 |
|---|---|---|
| `IR_KEY_POWER` | `0x28` | 원격 ESTOP 토글 |
| `IR_KEY_UP` | `0xC0` | 전진 (deadman) |
| `IR_KEY_DOWN` | `0x40` | 후진 (deadman) |
| `IR_KEY_LEFT` | `0x70` | 제자리 좌회전 |
| `IR_KEY_RIGHT` | `0x58` | 제자리 우회전 |
| `IR_KEY_NOZZLE_UP` | `0x08` | 노즐 UP (실측 후 교체) |
| `IR_KEY_NOZZLE_DOWN` | `0x88` | 노즐 DOWN (실측 후 교체) |

제어 우선순위:

1. 서버 ESTOP / CLEAR (`0xA55A`)
2. 리모컨 POWER (REMOTE latch만 토글, 서버 ESTOP은 해제 불가)
3. AUTO 서버 SET_SPEED / NOZZLE
4. MANUAL 리모컨 방향·노즐

## 6. 노즐 서보

| 서보선 | 연결 |
|---|---|
| Signal | PA8 / D7 / CN10-23 |
| V+ | 별도 5~6 V servo buck (Nucleo 5V 직접 공급 비권장) |
| GND | servo buck GND + 시스템 공통 GND |

서보 근처에 470~1000 uF bulk capacitor를 둡니다.
`ROBOT_SERVO_OFF_US` / `ROBOT_SERVO_ON_US`로 기구에 맞게 조정합니다.

## 7. 전원 구성

### 7.1 실험실(벤치) 전원

```text
PC USB ─────────────── NUCLEO-F401RE
RPi 어댑터 ─────────── Raspberry Pi
12 V 전원장치 ─┬────── 좌측 DRV8825 VMOT
               └────── 우측 DRV8825 VMOT
(선택) 5~6 V ───────── 서보 V+
Nucleo 3V3 ─────────── IR VCC

공통 GND: Nucleo ─ RPi ─ DRV×2 ─ 서보 ─ IR
```

### 7.2 정식(배터리) 전원

```text
배터리
  │
  ├─ 메인 퓨즈
  │
  ├─ Motor rail ─ 좌우 DRV8825 VMOT
  │
  ├─ 5.1 V 고전류 buck ─ Raspberry Pi
  │
  ├─ 5 V logic rail ─ Nucleo (+ IR 3V3는 Nucleo regulator)
  │
  └─ 5~6 V servo buck ─ 노즐 서보

모든 low-voltage rail GND는 설계된 한 지점에서 공통화
```

권장 보호: 메인/가지 퓨즈, 역극성 보호, TVS, bulk cap, buck I/O cap,
저전압 차단, 방열·팬, 잠금형 XT30/XT60 또는 screw terminal.

퓨즈·전선 굵기는 실제 모터 전류, 서보 stall, RPi 최대 부하, 배터리 단락
전류를 계산한 뒤 결정합니다.

### 7.3 배선 품질

- STEP/DIR과 모터 코일선을 나란히 길게 묶지 않습니다.
- 모터 코일 두 쌍은 각각 꼬아서 배선합니다.
- UART·IR 신호선은 짧게, 모터 전원과 떨어뜨립니다.
- 긴 케이블에는 strain relief를 적용합니다.
- 고전류 GND가 가는 점퍼선만으로 흐르지 않게 합니다.
- chassis와 신호 GND는 한 지점 접지 원칙으로 설계합니다.

## 8. 조립 및 시험 순서

### 단계 A: Nucleo만

1. 모터 전원·RPi를 연결하지 않습니다.
2. CubeIDE에서 빌드·다운로드합니다. (`.ioc` Generate는 하지 말고 먼저 Build)
3. USART2에서 부팅 문자열을 확인합니다.
4. PB0/PB2 STEP=LOW, PB6/PB7 EN=HIGH를 측정합니다.

### 단계 B: 좌측 모터

1. 전체 전원 OFF.
2. 좌측 STEP/DIR/EN/GND·코일·커패시터 연결.
3. 전류 제한을 낮게 두고 `+100 sps`를 100 ms 이하 주기로 전송.
4. 떨림·과열·방향을 확인합니다.

### 단계 C: 우측 모터

좌측 정상 후에만 추가. `+100,+100` → `+100,-100` → `-100,+100` 공중 시험.

### 단계 D: 서보

혼을 분리한 채 NOZZLE OFF/ON PWM 방향과 기구 한계를 확인합니다.

### 단계 E: RPi UART

1. 전원 OFF 후 TX/RX/GND 연결.
2. Nucleo 먼저 기동 후 RPi 실행.
3. AUTO에서 SET_SPEED를 100 ms 이하로 유지.
4. 300 ms 끊김 → ESTOP/TIMEOUT.
5. ESTOP → CLEAR는 완전 정지 후 `5A A5`만.

### 단계 F: IR

1. IR VCC/GND/OUT(PB4) 연결.
2. 키를 눌러 USART2 raw 코드를 확인하고 `ir_remote.h`를 맞춤.
3. AUTO에서 방향키는 거부, POWER는 원격 ESTOP.
4. `0x05` MANUAL 후 방향키 deadman·노즐 키 확인.
5. POWER 재입력이 서버 ESTOP을 풀지 못하는지 확인.

## 9. 아직 핀이 없는 향후 신호

다음은 **현재 미할당**입니다. 임의 GPIO에 연결하지 마십시오.

- 좌우 DRV8825 `FAULT`
- 배터리 전압 ADC
- 드라이버/모터 온도
- 문·커버 interlock

추가할 때는 CMD/STATUS 설계와 CubeMX pinout을 함께 변경합니다.
(IR는 이미 할당됨. 물리 ESTOP 핀은 사용하지 않음.)

## 10. CubeMX

### 결론

repository 상태 그대로면 **새로 켤 항목은 없습니다.**
TIM1/TIM2/TIM3/TIM5, PB4 IR, USART1/2, NVIC, FreeRTOS include가 이미
반영되어 있습니다.

첫 빌드:

1. `Paint_Robot/STM32` import
2. Generate Code **하지 말고** 먼저 Build
3. 최신 CubeIDE ARM GCC 사용
4. ST-LINK 다운로드 후 저속 시험

### Generate Code를 누를 경우

CMSIS-RTOS FreeRTOS wrapper를 추가하지 마십시오. Native kernel만
유지합니다.

확인 목록:

1. FreeRTOS include path / `Middlewares` source entry 유지
2. HAL Timebase = **TIM5** (TIM6 아님)
3. NVIC: TIM2=0, TIM3=6, USART1=6, USART2=7, SysTick/PendSV/TIM5=15
4. `TIM3_IRQHandler` 유지
5. TIM2 → `Motor_TickISR()`
6. UART callback → `UartTransport_*Callback()`
7. `HAL_TIM_MODULE_ENABLED`, `USE_RTOS=0`
8. `git diff` 후 빌드

## 11. 문제별 확인

### 모터가 안 움직임

VMOT, RESET/SLEEP HIGH, EN LOW, 코일 쌍, AUTO+SET_SPEED 주기,
Vref, ESTOP/MANUAL 여부

### 떨기만 함

코일 섞임, 가감속·속도, 전류, 전원 강하, microstep 1/16 일치

### 방향 반대

`ROBOT_*_FORWARD_LEVEL` 또는 코일 한 쌍 반전

### RPi 통신 안 됨

TX/RX 교차, GND, 115200-8-N-1, `/dev/ttyAMA0`, console off, `0xAA` 확인

### 연결 직후 ESTOP

SET_SPEED를 한 번만 보내지 않았는지, `0,0` heartbeat, CRC/LEN

### IR 무반응

3V3/GND/OUT(PB4), USART2 로그, TIM3 동작, 키 매크로

### MANUAL인데 안 움직임

`0x05` MANUAL(STATUS bit6), ESTOP latch 없음, deadman repeat,
AUTO가 아닌 것 확인

### POWER로 ESTOP이 안 풀림

서버 ESTOP이 남아 있으면 POWER는 REMOTE만 지웁니다. 서버는
`CLEAR_ESTOP(5A A5)`가 필요합니다.

## 12. 참고 자료

- [ST UM1724 NUCLEO-64](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [TI DRV8825](https://www.ti.com/lit/ds/symlink/drv8825.pdf)
- [Pololu DRV8825 carrier](https://www.pololu.com/product-info-merged/2133)
- 프로젝트 `MOTOR_SETUP_KR.md` (UART·FreeRTOS·시험 정본)
- 프로젝트 `Robot_Painter.ioc`
- IR 예제 원본: `ir_mod/stm32_firmware/Core`
