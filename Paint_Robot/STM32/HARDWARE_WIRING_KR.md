# V-Road-Painter 하드웨어 배선 가이드

이 문서는 현재 펌웨어의 핀맵을 기준으로 다음 두 구성을 설명합니다.

- 데모 버전: NUCLEO-F401RE, Raspberry Pi, NEMA17 모터 2개,
  DRV8825 2개, CNI DRV8825 확장보드 2개
- 정식 버전: 배터리, 전원 분배, 물리 ESTOP, 서보 노즐 및 보호회로까지 포함

확장보드마다 실크 인쇄가 조금씩 다를 수 있습니다. `STEP/STP`, `EN/ENABLE/nENBL`,
`M0/MS1`처럼 이름만 다른 경우가 있으므로 반드시 실제 보드의 인쇄와 회로를
확인한 뒤 연결하십시오.

## 1. 가장 중요한 안전 규칙

1. 모터 전원이 켜진 상태에서 모터 또는 DRV8825를 꽂거나 빼지 않습니다.
2. DRV8825 방향을 반대로 꽂으면 Nucleo, 드라이버, Raspberry Pi가 파손될 수
   있습니다. `VMOT`, `GND`, 가변저항 위치를 먼저 확인합니다.
3. Nucleo의 `5V`, `3V3` 핀으로 NEMA17 모터에 전원을 공급하지 않습니다.
4. Raspberry Pi의 `5V` 또는 `3V3` 핀으로 모터에 전원을 공급하지 않습니다.
5. DRV8825의 모터 전원은 별도 전원장치에서 공급합니다.
6. RPi, Nucleo, 좌우 DRV8825 전원의 GND는 반드시 공통으로 연결합니다.
7. VMOT 바로 옆에 드라이버마다 최소 100 uF 전해 커패시터를 설치합니다.
8. 첫 시험은 바퀴를 바닥에서 띄우고 낮은 전류와 `100 sps` 이하로 진행합니다.
9. 정식 장비의 ESTOP은 UART 명령만 사용하지 말고 모터 전원을 물리적으로
   차단할 수 있어야 합니다.

## 2. NUCLEO-F401RE 현재 펌웨어 핀맵

아래 핀은 `Robot_Painter.ioc`, `main.h`, `gpio.c`, `usart.c`의 현재 설정과
일치합니다.

| 용도 | STM32 GPIO | Nucleo에서 찾기 쉬운 이름 | ST Morpho 위치 | 연결 대상 |
|---|---:|---|---|---|
| 좌측 STEP | PB0 | Arduino `A3` | `CN7-34` | 좌측 보드 `STEP` |
| 좌측 DIR | PB1 | Arduino 헤더 없음 | `CN10-24` | 좌측 보드 `DIR` |
| 좌측 ENABLE | PB6 | Arduino `D10` | `CN10-17` | 좌측 보드 `EN/nENBL` |
| 우측 STEP | PB2 | Arduino 헤더 없음 | `CN10-22` | 우측 보드 `STEP` |
| 우측 DIR | PB5 | Arduino `D4` | `CN10-29` | 우측 보드 `DIR` |
| 우측 ENABLE | PB7 | Arduino 헤더 없음 | `CN7-21` | 우측 보드 `EN/nENBL` |
| 노즐 서보 신호 | PA8 | Arduino `D7` | `CN10-23` | 서보 signal |
| STM32 USART1 TX | PA9 | Arduino `D8` | `CN10-21` | RPi RXD GPIO15 |
| STM32 USART1 RX | PA10 | Arduino `D2` | `CN10-33` | RPi TXD GPIO14 |

전원 및 GND:

- Nucleo `3V3`: `CN7-16`
- Nucleo `5V`: `CN7-18`
- Nucleo GND: Arduino GND 또는 Morpho `CN7-8/19/20/22`,
  `CN10-9/20` 중 하나

Morpho 커넥터 번호는 보드를 위에서 보고 USB 커넥터 방향을 확인한 뒤 공식
NUCLEO-64 사용자 매뉴얼의 `CN7`, `CN10` 번호와 대조하십시오. GPIO 이름을
확인하지 않고 단순히 “몇 번째 핀”만 세어 연결하면 보드 방향에 따라 반대로
셀 수 있습니다.

## 3. 데모 버전 전체 배선

### 3.1 데모에 필요한 추가 부품

현재 가지고 있는 부품 외에 다음이 필요합니다.

- 모터용 DC 전원장치
  - 첫 시험 권장: 전류 제한 가능한 12 V 전원장치
  - 모터 2개 시험 시 최소 3~5 A급에서 낮은 제한값부터 시작
- DRV8825별 100~470 uF 전해 커패시터 1개
  - 12 V 사용 시 25 V 이상 정격 권장
  - 24 V 사용 시 35 V 이상 정격 권장
- 점퍼선, 굵은 모터 전원선, 퓨즈 또는 재설정형 차단기
- 멀티미터
- 가능하면 오실로스코프 또는 로직 애널라이저
- DRV8825 방열판 및 필요 시 냉각팬

### 3.2 Nucleo와 좌측 DRV8825 확장보드

| Nucleo | 좌측 확장보드 | 설명 |
|---|---|---|
| PB0 / A3 / CN7-34 | `STEP` 또는 `STP` | 상승 에지마다 1 microstep |
| PB1 / CN10-24 | `DIR` | 회전 방향 |
| PB6 / D10 / CN10-17 | `EN`, `ENABLE`, `nENBL` | LOW 활성, HIGH 비활성 |
| GND | Logic GND | 신호 기준 GND |

부팅 시 펌웨어는 `EN=HIGH`로 만들어 드라이버를 비활성화합니다. 유효한
SET_SPEED 명령이 들어오면 LOW로 바뀝니다.

### 3.3 Nucleo와 우측 DRV8825 확장보드

| Nucleo | 우측 확장보드 | 설명 |
|---|---|---|
| PB2 / CN10-22 | `STEP` 또는 `STP` | 상승 에지마다 1 microstep |
| PB5 / D4 / CN10-29 | `DIR` | 회전 방향 |
| PB7 / CN7-21 | `EN`, `ENABLE`, `nENBL` | LOW 활성, HIGH 비활성 |
| GND | Logic GND | 신호 기준 GND |

### 3.4 DRV8825 공통 설정

좌우 보드 모두 다음과 같이 설정합니다.

| DRV8825 단자 | 연결 |
|---|---|
| `nRESET`, `RESET`, `RST` | HIGH 유지 |
| `nSLEEP`, `SLEEP`, `SLP` | HIGH 유지 |
| `MODE0/M0/MS1` | LOW |
| `MODE1/M1/MS2` | LOW |
| `MODE2/M2/MS3` | HIGH |
| `FAULT` | 현재 데모에서는 연결하지 않음 |

현재 펌웨어의 `ROBOT_MICROSTEP_DIVISOR`는 16이므로
`MODE0=LOW`, `MODE1=LOW`, `MODE2=HIGH`인 1/16 microstep이어야 합니다.

확장보드에서 microstep jumper가 HIGH에 연결되는 방식이라면 M2 위치에만
jumper를 설치합니다. 보드에 DIP switch가 있으면 M2만 ON으로 둡니다.
보드 회로가 반대 방식일 수도 있으므로 전원이 꺼진 상태에서 continuity 측정으로
jumper가 실제로 HIGH에 연결되는지 확인하십시오.

`RESET`과 `SLEEP`은 둘 다 HIGH여야 동작합니다. 확장보드가 이미 pull-up 또는
jumper로 HIGH를 제공하면 추가 배선이 필요 없습니다. 직접 연결해야 한다면 두
핀을 묶어 Nucleo `3V3`에 연결할 수 있습니다. 5 V로 HIGH를 만들더라도 그 전압을
STM32 GPIO로 되돌려 연결하면 안 됩니다.

### 3.5 모터 전원 배선

각 드라이버:

| 전원장치 | DRV8825 |
|---|---|
| DC `+` | `VMOT`, `M+`, `V+` |
| DC `-` | `GND`, `M-`, `V-` |

커패시터:

- `+` 다리: VMOT
- `-` 다리: 모터 GND
- 드라이버 단자 바로 옆에 배치

좌우 드라이버를 같은 전원장치에서 사용하면 전원 분배점에서 각각 별도 가지로
분기합니다. 한 드라이버 단자를 거쳐 다른 드라이버로 직렬처럼 이어 붙이지
않는 것이 좋습니다.

### 3.6 NEMA17 모터 코일 연결

CIH-4248은 2상 4선 bipolar stepper motor입니다.

1. 모터를 전원과 완전히 분리합니다.
2. 멀티미터 저항 또는 continuity 모드로 서로 연결된 두 선을 찾습니다.
3. 첫 코일 쌍을 `A1/A2` 또는 `1A/1B`에 연결합니다.
4. 나머지 코일 쌍을 `B1/B2` 또는 `2A/2B`에 연결합니다.

한 코일의 선 하나와 다른 코일의 선 하나를 같은 A 쌍에 섞으면 모터가 떨기만
하고 회전하지 않습니다.

회전 방향이 반대인 경우:

- 가장 권장: `robot_config.h`의 해당 `ROBOT_*_FORWARD_LEVEL` 변경
- 또는 전원을 끈 뒤 한 코일 쌍의 두 선만 서로 교환

### 3.7 DRV8825 전류 제한

모터 정격이 1.8 A/phase라고 해서 처음부터 1.8 A로 설정하면 안 됩니다.
DRV8825 clone마다 sense resistor 값과 Vref 계산식이 다를 수 있습니다.

1. 실제 보드의 `R050`, `R100`, `R200` 등 sense resistor 표기를 확인합니다.
2. 확장보드 또는 판매처의 Vref 계산식을 확인합니다.
3. 첫 시험은 약 0.5 A 수준의 낮은 제한부터 시작합니다.
4. 탈조, 토크, 모터 온도, 드라이버 온도를 확인하며 단계적으로 올립니다.
5. 1.8 A 근처는 방열판과 강제 냉각이 필요할 수 있습니다.

Vref를 측정할 때 프로브로 주변 핀을 단락시키지 않도록 절연 드라이버를
사용하십시오.

### 3.8 Raspberry Pi와 Nucleo UART

Raspberry Pi 40-pin header:

| Raspberry Pi | Nucleo | 방향 |
|---|---|---|
| 물리 8번, GPIO14 TXD | PA10 / D2 / CN10-33 | RPi → STM32 |
| 물리 10번, GPIO15 RXD | PA9 / D8 / CN10-21 | STM32 → RPi |
| 물리 6번 GND | Nucleo GND | 공통 기준 |

주의:

- TX와 RX는 서로 교차합니다.
- 두 장치 모두 3.3 V UART이므로 레벨시프터는 필요 없습니다.
- UART 배선에 RPi 5 V를 연결하지 않습니다.
- 데모에서는 RPi는 자체 USB-C 전원, Nucleo는 ST-LINK USB 전원을 사용하고
  두 보드의 GND만 공통으로 만드는 방법이 안전합니다.
- Raspberry Pi serial console을 비활성화하고 hardware UART를 활성화해야 합니다.
- 현재 RPi 코드는 `/dev/ttyAMA0`, 115200-8-N-1을 사용하므로 실제 보드에서
  `/dev/ttyAMA0`이 GPIO14/15에 연결되는지 확인하십시오.

### 3.9 데모에서는 연결하지 않는 것

현재 보유 부품만으로 모터 통신 데모를 할 때 다음은 비워 둡니다.

- PA8 서보 신호
- DRV8825 FAULT
- 배터리 전압 감지
- 장애물 센서
- 물리 ESTOP 입력

사용하지 않는 입력을 임의 GPIO에 연결하면 현재 펌웨어와 충돌할 수 있습니다.

## 4. 데모 전원 구성

```text
PC USB ─────────────── NUCLEO-F401RE
RPi 전원 어댑터 ───── Raspberry Pi
12 V 전원장치 ─┬──── 좌측 DRV8825 VMOT
               └──── 우측 DRV8825 VMOT

Nucleo GND ─ RPi GND ─ 좌측 DRV GND ─ 우측 DRV GND
```

모터 전원장치의 `+`는 Nucleo나 RPi에 직접 연결하지 않습니다. 공통으로 연결하는
것은 GND입니다.

## 5. 데모 조립 및 시험 순서

### 단계 A: Nucleo만 시험

1. 모터 전원과 RPi를 연결하지 않습니다.
2. Nucleo를 USB로 연결하고 STM32CubeIDE에서 빌드·다운로드합니다.
3. USART2 ST-LINK Virtual COM에서 부팅 문자열을 확인합니다.
4. PB0/PB2 STEP이 LOW, PB6/PB7 ENABLE이 HIGH인지 측정합니다.

### 단계 B: 좌측 모터 하나만 시험

1. 전체 전원을 끕니다.
2. 좌측 STEP/DIR/EN/GND를 연결합니다.
3. 모터 코일과 100~470 uF 커패시터를 연결합니다.
4. 전원장치 전류 제한을 낮게 설정합니다.
5. 모터 전원을 켠 뒤 `+100 sps`를 100 ms 이하 주기로 전송합니다.
6. 떨림, 과열, 잘못된 방향을 확인합니다.

### 단계 C: 우측 모터 추가

좌측이 정상일 때만 같은 방식으로 우측을 추가합니다. 좌우 `+100,+100`,
`+100,-100`, `-100,+100` 순서로 공중 시험합니다.

### 단계 D: Raspberry Pi 연결

1. 모든 전원을 끕니다.
2. TX/RX/GND를 연결합니다.
3. Nucleo를 먼저 켜고 RPi 프로그램을 실행합니다.
4. SET_SPEED를 100 ms 이하 주기로 계속 보냅니다.
5. 300 ms 이상 중단했을 때 ESTOP/TIMEOUT STATUS가 오는지 확인합니다.

### 단계 E: 안전 명령

1. 주행 중 ESTOP을 보냅니다.
2. 노즐이 OFF 상태로 유지되는지 확인합니다.
3. 완전히 정지하기 전에 CLEAR_ESTOP이 거부되는지 확인합니다.
4. 정지 후 `0xA55A` key로만 해제되는지 확인합니다.

## 6. 정식 버전 전원 구조

정식 로봇에서는 한 전원에서 그대로 병렬로 뽑지 말고 기능별 전원 rail로
분리합니다.

```text
배터리
  │
  ├─ 메인 퓨즈
  │
  ├─ 물리 ESTOP / contactor
  │    └─ Motor rail ─ 좌우 DRV8825 VMOT
  │
  ├─ 5.1 V 고전류 buck ─ Raspberry Pi
  │
  ├─ 5 V logic rail ─ Nucleo
  │
  └─ 5~6 V servo buck ─ 노즐 서보

모든 low-voltage rail GND는 설계된 한 지점에서 공통화
```

### 6.1 권장 보호 부품

- 배터리 직후 메인 퓨즈
- 좌우 드라이버 가지별 퓨즈
- 역극성 보호
- TVS diode 또는 적절한 surge 대책
- 드라이버별 bulk capacitor
- 5 V buck 입력·출력 capacitor
- 배터리 저전압 차단
- 방열판과 온도에 맞춘 팬
- 잠금형 XT30/XT60 또는 screw terminal

퓨즈 정격과 전선 굵기는 실제 모터 전류, 서보 stall current, RPi 최대 부하,
배터리 단락 전류를 계산한 뒤 결정해야 합니다.

### 6.2 물리 ESTOP

현재 UART `0x03 ESTOP`은 제어 시스템의 소프트웨어 정지입니다. 정식 장비는
버튼 고장, RPi 정지, STM32 fault에도 모터 토크를 제거할 수 있어야 합니다.

권장 구조:

- NC 방식 ESTOP 스위치
- contactor 또는 정격이 충분한 motor rail 차단회로
- 보조 접점을 STM32/RPi 입력에도 전달해 상태 기록
- ESTOP 해제 후 자동 재시작 금지
- 물리 해제와 소프트웨어 CLEAR_ESTOP을 모두 확인한 뒤 재가동

DRV8825 `EN`만 HIGH로 만드는 것은 제어 신호 차단이며 안전 등급의 전원 차단을
대체하지 않습니다.

### 6.3 노즐 서보

정식 버전에서 서보를 추가할 때:

| 서보선 | 연결 |
|---|---|
| Signal | PA8 / D7 / CN10-23 |
| V+ | 별도 5~6 V servo buck |
| GND | servo buck GND 및 시스템 공통 GND |

서보 전원을 Nucleo 5 V pin에서 직접 공급하지 않는 것이 좋습니다. 서보 stall
current를 감당할 buck converter를 사용하고 서보 가까이에 470~1000 uF
capacitor를 배치합니다.

### 6.4 배선 품질

- STEP/DIR과 모터 코일선을 나란히 길게 묶지 않습니다.
- 모터 코일 두 쌍은 각각 꼬아서 배선합니다.
- UART는 짧게 유지하고 모터 전원선과 떨어뜨립니다.
- 긴 케이블에는 connector strain relief를 적용합니다.
- 고전류 GND와 logic GND가 가는 점퍼선을 통해 흐르지 않게 합니다.
- chassis와 신호 GND 연결은 한 지점 접지 원칙으로 설계합니다.

### 6.5 향후 추가를 권장하는 신호

정식 버전에서는 다음을 고려할 수 있지만 현재 펌웨어에는 아직 핀이 할당되지
않았습니다.

- 좌우 DRV8825 `FAULT`
- 물리 ESTOP 상태 입력
- 배터리 전압 ADC
- 드라이버 또는 모터 온도
- 문/커버 interlock

핀을 먼저 임의로 연결하지 말고 새 CMD/STATUS 설계와 CubeMX pin allocation을
함께 변경해야 합니다.

## 7. CubeMX에서 추가로 설정해야 하는가?

### 결론

현재 repository 상태 그대로 사용할 경우 CubeMX에서 새로 설정할 항목은
없습니다. 핀, TIM1/TIM2/TIM5, USART1/USART2, NVIC priority, FreeRTOS kernel
include path가 이미 코드와 프로젝트에 반영되어 있습니다.

첫 빌드 절차:

1. STM32CubeIDE에서 `Paint_Robot/STM32` 프로젝트를 import합니다.
2. `.ioc`를 Generate Code 하지 말고 먼저 기존 프로젝트를 Build합니다.
3. 최신 STM32CubeIDE ARM GCC toolchain을 사용합니다.
4. Build가 성공하면 ST-LINK로 다운로드합니다.
5. 데모 배선 후 저속 시험합니다.

### “무조건 바로 빌드된다”고 단정할 수 없는 이유

현재 로컬 PATH의 `arm-none-eabi-gcc 4.4.1`은 최신 CMSIS를 컴파일할 수 없어
전체 link build를 검증하지 못했습니다. 전처리, FreeRTOS kernel 구문 검사,
protocol test는 통과했지만 최신 STM32CubeIDE에서 실제 Build 확인이 한 번
필요합니다.

### CubeMX Generate Code를 누를 경우

이 프로젝트는 FreeRTOS Native kernel을 source-controlled middleware로
관리합니다. CubeMX에서 CMSIS-RTOS FreeRTOS middleware를 다시 활성화하면
wrapper와 task 코드가 중복될 수 있으므로 추가하지 마십시오.

Generate Code 후 반드시 확인할 것:

1. `.cproject`에 다음 include path가 남아 있는지 확인
   - `Middlewares/Third_Party/FreeRTOS/Source/include`
   - `Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F`
2. `.cproject` source entry에 `Middlewares`가 남아 있는지 확인
3. HAL Timebase Source가 TIM5인지 확인
4. NVIC가 TIM2=0, USART1=6, USART2=7, SysTick/PendSV/TIM5=15인지 확인
5. `stm32f4xx_it.c`에서 SVC/PendSV/SysTick이 중복 정의되지 않는지 확인
6. TIM2 handler가 `Motor_TickISR()`을 호출하는지 확인
7. `main.c` UART callbacks가 `UartTransport_*Callback()`으로 전달되는지 확인
8. `stm32f4xx_hal_conf.h`에서 `HAL_TIM_MODULE_ENABLED`가 활성인지 확인
9. `USE_RTOS=0`인지 확인

이 HAL release는 `USE_RTOS=1`을 허용하지 않으므로 0이 정상입니다.

## 8. 문제별 확인 방법

### 모터가 전혀 움직이지 않음

- VMOT 전압 확인
- RESET/SLEEP HIGH 확인
- EN이 명령 후 LOW인지 확인
- 코일 쌍이 올바른지 확인
- 300 ms watchdog 전에 SET_SPEED가 반복되는지 확인
- Vref가 지나치게 낮지 않은지 확인

### 모터가 떨기만 함

- A/B 코일 쌍이 섞였는지 확인
- 가속도 또는 속도를 낮춤
- 전류 제한을 조금 올림
- 전원 전압 강하 확인

### 방향이 반대임

- `robot_config.h`의 해당 forward level 변경
- 또는 전원 OFF 후 한 코일 쌍만 반전

### RPi 통신이 안 됨

- TX/RX 교차 확인
- 공통 GND 확인
- 115200-8-N-1 확인
- `/dev/ttyAMA0` 확인
- serial console 비활성화 확인
- 로직 애널라이저로 `0xAA` 시작 바이트 확인

### 연결 후 바로 ESTOP됨

- SET_SPEED를 한 번만 보내고 멈추지 않았는지 확인
- 정지 상태도 `0,0` SET_SPEED를 100 ms 이하 주기로 반복
- CRC와 LEN 확인

## 9. 참고 자료

- [ST UM1724 NUCLEO-64 사용자 매뉴얼](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
- [TI DRV8825 데이터시트](https://www.ti.com/lit/ds/symlink/drv8825.pdf)
- [Pololu DRV8825 carrier 설명](https://www.pololu.com/product-info-merged/2133)
- 프로젝트 내부 `MOTOR_SETUP_KR.md`
- 프로젝트 내부 `Robot_Painter.ioc`
