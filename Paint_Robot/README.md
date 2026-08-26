# Road Painter — AMR 하드웨어 & 임베디드 제어 시스템

본 디렉토리는 **Road Painter (인프라 비전 측위 기반 자율 노면 도장 로봇)**의 하드웨어(ME/HW), 하위 펌웨어(STM32 FreeRTOS), 상위 제어기(Raspberry Pi Linux LKM & C++17) 소스 코드 및 시스템 배포 가이드를 포함합니다.

---

##  1. 로봇 하드웨어 물리 사양 & 기구학 (Kinematics)

| 항목 (Specification) | 수치 / 규격 | 세부 비고 |
| :--- | :--- | :--- |
| **구동 방식** | **6WD 차동 구동 (Differential Drive)** | 2 능동 구동륜 + 4 수동 옴니휠 (4점 지지 평면) |
| **차체 외형 규격** | **$170 \times 170 \times 130\text{ mm}$** | 3D 프린팅 4단 적층 콤팩트 프레임 (자체 하중 ~1.2 kg) |
| **구동 휠 직경 ($D$)** | **$66.0\text{ mm}$ ($0.066\text{ m}$)** | 실측 휠 둘레 $\pi D \approx 207.345\text{ mm}$ |
| **트레드 윤거 ($W$)** | **$166.0\text{ mm}$ ($0.166\text{ m}$)** | 좌/우 구동 휠 중심 간 트랙 거리 |
| **마커-펜 오프셋 ($a$)** | **$155.0\text{ mm}$ ($0.155\text{ m}$)** | ArUco 마커 중심 ➔ 도색 펜 촉 중심 오프셋 |
| **스텝 분해능** | **$3,200\text{ pulses/rev}$ (1/16 Microstep)** | 1미터당 **$15,433.09\text{ pulses}$ ($64.8\text{ }\mu\text{m/step}$)** |
| **거리 보정 계수 ($K_{\text{dist}}$)**| **`1.021f`** | 실측 $15\text{cm}$ 주행 오차 보정 완료 ($15,757\text{ pulses/m}$) |
| **회전 보정 계수 ($K_{\text{turn}}$)**| **`1.050f`** | 옴니휠 횡슬립 보정 완료 ($2,113\text{ pulses/90}^\circ$, $23.48\text{ pulses/deg}$) |
| **정격 주행 속도** | **직진 $0.05\text{ m/s}$ (771 SPS)** / **회전 $16.5^\circ/\text{s}$ (400 SPS)** | 수동 조작: 직진 500 SPS / 회전 300 SPS |
| **메인 전원** | **4S Li-Po (14.8V 2200mAh)** | XL4015 DC-DC 5V 5A 강압 (로직 및 서보/앰프 분배) |

---

##  2. 시스템 아키텍처 및 디렉토리 구조

```text
Paint_Robot/
├── RaspberryPi/                    ➔ [상위 제어기: Linux MPU]
│   ├── driver/                     • 리눅스 커널 모듈(LKM) 3종 & DTS 오버레이
│   │   ├── audio_strip_driver.c    - BCM2711 PCM/I2S DMA 오디오 드라이버 (/dev/audio_strip)
│   │   ├── led_strip_driver.c      - BCM2711 PWM0-DMA WS2812B LED 드라이버 (/dev/led_strip)
│   │   ├── mpu6050_driver.c        - kthread 500Hz I2C 자이로 적산 드라이버 (/dev/mpu6050)
│   │   └── Makefile                - 커널 모듈 및 DTS 통합 빌드
│   ├── audio/                      • 현장 음성 안내 & 경보 시스템 (10종 WAV 카탈로그)
│   ├── include/ & src/             • C++17 메인 제어기 (Network, PathFollower, Serial, Audio, LED, IMU)
│   ├── daemon/                     • systemd 자동 실행 서비스 (robot_exec.service)
│   └── CMakeLists.txt              • RPi C++ 통합 빌드 스크립트
│
└── STM32/                          ➔ [하위 제어기: ARM Cortex-M4 MCU]
    └── Core/
        ├── Inc/ & Src/
        │   ├── motor.c / .h        • 20 kHz TIM2 하드웨어 ISR Bresenham 선형 가감속 엔진
        │   ├── servo.c / .h        • 50 Hz TIM1 PWM MG996R 노즐 리프팅 (동적 펄스 튜닝)
        │   ├── uart_protocol.c     • 1바이트 패킹 바이너리 프로토콜 직렬화/역직렬화/체크섬
        │   ├── control_arbiter.c   • 3단계 제어권 중재기 (ESTOP > IR수동 > 자율주행) & 300ms 워치독
        │   └── app_rtos.c          • FreeRTOS 스케줄러 (ControlTask +3 / CommTask +2)
        └── Robot_Painter.ioc       • STM32CubeMX 하드웨어 핀맵 및 페리페럴 설정
```

---

##  3. 하드웨어 배선 및 핀맵 요약

### 1) STM32F401RE 핀맵
| 모듈 / 핀 | 기능 | STM32 핀 | 페리페럴 / 모드 | 신호 규격 |
| :--- | :--- | :--- | :--- | :--- |
| **좌측 모터** | STEP / DIR / EN | `PB0` / `PB1` / `PB6` | `TIM2_CH3` / GPIO OUT | 20 kHz 펄스 / Active-Low |
| **우측 모터** | STEP / DIR / EN | `PB2` / `PB5` / `PB7` | `TIM2_CH4` / GPIO OUT | 20 kHz 펄스 / Active-Low |
| **노즐 서보** | PWM | `PA8` | `TIM1_CH1` | 50 Hz PWM (UP: 1600µs, DOWN: 1300µs) |
| **적외선 리모컨** | IR Signal | `PB4` | `TIM3_CH1` | NEC 적외선 디코딩 |
| **RPi 통신** | UART RX / TX | `PA10` / `PA9` | `USART1` | 115200 bps, 8N1, 바이너리 패킷 |

### 2) Raspberry Pi 4B GPIO 핀맵
| 기능 | GPIO 번호 | 물리 핀 번호 | 핀 모드 | 연결 모듈 |
| :--- | :--- | :--- | :--- | :--- |
| **I2S BCLK** | GPIO 18 | Pin 12 | `ALT0 (PCM_CLK)` | MAX98357A I2S 앰프 |
| **I2S LRC** | GPIO 19 | Pin 35 | `ALT0 (PCM_FS)` | MAX98357A I2S 앰프 |
| **I2S DIN** | GPIO 21 | Pin 40 | `ALT0 (PCM_DOUT)`| MAX98357A I2S 앰프 |
| **LED 데이터** | GPIO 12 | Pin 32 | `ALT0 (PWM0_0)` | WS2812B 5구 LED 스트립 |
| **IMU I2C** | GPIO 2, 3 | Pin 3, 5 | `I2C1 (SDA/SCL)`| MPU-6050 자이로 센서 |
| **STM32 UART**| GPIO 14, 15 | Pin 8, 10 | `ALT0 (TXD/RXD)` | STM32 USART1 (`PA10`/`PA9`) |

---

##  4. 원클릭 설치 및 실행 가이드

### [Step 1] Raspberry Pi 소프트웨어 빌드
```bash
# 1. 라즈베리파이 접속
ssh user@<RPI_IP>

# 2. 저장소 빌드
cd ~/Painter_Robot
mkdir -p build && cd build
cmake ..
make -j4
```

### [Step 2] 커널 드라이버 & 오디오 모듈 설치
```bash
# 1. 오디오 커널 드라이버 및 WAV 음원 전역 배포
cd ~/Painter_Robot/audio
sudo ./install_audio.sh

# 2. LED 및 IMU 커널 드라이버 빌드 & 설치 (최초 1회)
cd ~/Painter_Robot/driver
make all
sudo insmod led_strip_driver.ko
sudo insmod mpu6050_driver.ko
```

### [Step 3] 시스템 서비스 데몬 등록 (부팅 시 자동 실행)
```bash
cd ~/Painter_Robot/daemon
sudo ./install_daemon.sh
```

---

##  5. 테스트 및 수동 검증 도구

### 1) 단독 캘리브레이션 툴 (`calibration_test`)
```bash
cd ~/Painter_Robot/build
./calibration_test /dev/serial0
```
* `move 0.15` : 15cm 직진 주행 테스트
* `turn 90` : 90도 제자리 회전 테스트
* `nozzle 1` / `nozzle 0` : 노즐 펜 하강(스무스 랜딩)/상승 테스트
* `status` : STM32 실시간 스텝수 및 플래그 확인

### 2) I2S 오디오 음향 테스트 (`play.sh`)
```bash
cd ~/Painter_Robot/audio
./play.sh task_start       # 주행 시작 음성
./play.sh emergency_stop   # 비상 정지 경보음
./play.sh person_in_zone   # 침입 경보 사이렌
```

### 3) 메인 자율주행 제어기 수동 실행 (`robot_exec`)
```bash
cd ~/Painter_Robot/build
./robot_exec 192.168.0.8    # 중앙 관제 서버 IP 지정
```
