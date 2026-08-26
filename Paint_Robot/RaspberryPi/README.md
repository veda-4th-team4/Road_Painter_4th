# Road-Painter Raspberry Pi Controller (Protocol v2)

라즈베리파이 4B(로봇 MPU)에서 동작하는 C++ 기반 메인 로봇 제어기 및 네트워크/시리얼 게이트웨이 프로그램입니다. 비전 서버로부터 Wi-Fi(TLS 1.2 소켓)를 통해 **Server-Master Protocol v2 (`ops` 배열, `op_index` 트랜잭션 매칭, `READY`, `GO`, `ALIGN`, `MORE`, `DRIFT`, `HOLD`, `PATH_DONE`)**를 수신하고, 스텝 오도메트리 제어 루프를 통해 STM32 모터 제어기로 UART 속도/노즐 지령을 송신하는 역할을 수행합니다.

---

## 🛠️ 사전 요구사항 및 의존성 설치 (최초 1회)

RPi 내에서 SSL 암호화 소켓 연동, I2C IMU 제어 및 시리얼 통신 빌드를 진행하기 위해 아래 패키지들을 사전에 설치합니다.

```bash
sudo apt-get update
sudo apt-get install -y g++ make cmake libssl-dev wiringpi raspi-gpio
```

---

## 📂 프로젝트 구조

```text
Paint_Robot/RaspberryPi/
├── driver/                         ➔ [리눅스 커널 공간: LKM & Device Tree]
│   ├── audio_strip_driver.c        • BCM2711 PCM/I2S + DMA 직결 오디오 커널 드라이버
│   ├── audio_strip_regs.h          • PCM/I2S 레지스터 맵 및 ioctl(DRAIN/DROP) 정의
│   ├── audio_strip_overlay.dts     • GPIO 18/19/21 I2S ALT0 핀먹스 오버레이 (/dev/audio_strip)
│   ├── led_strip_driver.c          • BCM2711 PWM0 Serializer + DMA 직결 커널 드라이버
│   ├── led_strip_regs.h / .dts     • GPIO 12(PWM0_0) 핀먹스 오버레이 (/dev/led_strip)
│   ├── mpu6050_driver.c / .dts     • kthread 기반 500 Hz I2C 폴링 IMU 드라이버 (/dev/mpu6050)
│   └── Makefile                    • 커널 모듈 통합 빌드 스크립트
│
├── audio/                          ➔ [현장 음성 알림 시스템]
│   ├── install_audio.sh / play.sh  • I2S 오디오 시스템 자동 설치 및 단독 테스트 스크립트
│   └── wav_files/*.wav             • 44.1kHz 16-bit stereo 표준 음원 카탈로그 10종
│
├── include/ & src/                 ➔ [유저 공간: C++17 POSIX Core Application]
│   ├── AudioStripManager (.h/.cpp) • 비동기 WAV 스트리밍 & 우선순위 선점(Preemption) 오디오 매니저
│   ├── LedStripManager (.h/.cpp)   • std::thread 백그라운드 30 fps 상태 렌더러 (/dev/led_strip)
│   ├── NetworkManager (.h/.cpp)    • 중앙 서버 TLS 1.2 소켓 통신, JSON 파싱, 침입 경보 수신
│   ├── PathFollower (.h/.cpp)      • 6WD 역기구학 연산, 1단 감속(8%), 스텝 오도메트리 추종
│   ├── SerialManager (.h/.cpp)     • termios 기반 Non-blocking UART 통신 (115200 bps)
│   ├── ImuManager (.h/.cpp)        • /dev/mpu6050 캐릭터 디바이스 제어 및 동적 영점 리셋
│   └── main.cpp                    • 메인 제어 루프, 50 Hz 상태 머신 스케줄링 및 Failsafe
│
└── daemon/                         ➔ [시스템 운영 및 자동화: systemd Service]
    ├── robot_exec.service          • 부팅 시 자동 실행 및 3초 이내 자동 재기동 (Restart=always)
    └── install_daemon.sh           • 원클릭 systemd 데몬 등록 및 권한 자동화 스크립트
```

---

## 📐 하드웨어 기구학 규격 & 보정 계수

* **바퀴 지름 (Wheel Diameter)**: $66\text{ mm}$ ($0.066\text{ m}$)
* **차축 거리 (Wheelbase)**: $166\text{ mm}$ ($0.166\text{ m}$)
* **1회전당 스텝 수**: $3,200\text{ steps/rev}$ (1/16 마이크로스텝)
* **직진 보정 계수 (`DISTANCE_CALIB_FACTOR`)**: `1.021f` $\rightarrow$ 약 $15,757\text{ steps/m}$ ($10\text{cm} = \mathbf{1,576\text{ 스텝}}$)
* **회전 보정 계수 (`TURN_ANGLE_CALIB_FACTOR`)**: `1.050f` $\rightarrow$ 약 $2,113\text{ steps/90}^\circ$ ($1^\circ = \mathbf{23.48\text{ 스텝}}$)
* **서보 펄스 폭 (PWM)**: UP = `1600 µs`, DOWN = `1300 µs` (스무스 랜딩 완충 하강 적용)

---

## 🏗️ 빌드 가이드 (Build Guide)

라즈베리파이 터미널(SSH)에 접속한 뒤 아래 명령어로 빌드합니다.

```bash
cd ~/Painter_Robot
mkdir -p build && cd build
cmake ..
make -j4
```
* 빌드가 완료되면 `build` 폴더 내에 메인 제어 바이너리 **`robot_exec`**, 캘리브레이션 툴 **`calibration_test`**, 오디오 테스트 툴 **`audio_strip_test`**가 생성됩니다.

---

## 🚀 실행 및 테스트 가이드

### 1) 로봇 메인 제어 루프 구동 (`robot_exec`)
```bash
cd ~/Painter_Robot/build
./robot_exec 192.168.0.8    # 중앙 서버 IP 입력
```

* **주요 기능 및 안전 인터락**:
  1. **`op_index` 트랜잭션 검증**: 수신되는 모든 `GO`, `ALIGN`, `MORE`, `DRIFT` 명령의 `op_index`를 엄격 대조하여 지연 응답에 의한 오작동 완전 차단.
  2. **`ALIGN` / `MORE` 2초 안착 딜레이**: 미세 회전 및 미세 전진/후진 완료 후 차체/카메라 진동 방지를 위해 2.0초(2000ms) 강제 안착 대기 후 서버로 `READY` 전송.
  3. **`DRIFT` 주행 중 연속 차동 조향**: 직진 도색 주행 중 멈추지 않고 실시간으로 좌/우 바퀴 차동 속도를 제어하여 보정 조향.
  4. **`HOLD` 자동 일시정지 및 재개**: 비전 위치 두절($\ge 2$초) 시 `HOLD: PAUSE` 수신으로 즉시 정지, 회복 시 `HOLD: RESUME`로 멈췄던 자리에서 주행 자동 재개.
  5. **현장 시각·청각 안전 알림**:
     - **I2S 스피커**: 부팅(`POWER_ON`), 주행시작(`TASK_START`), 침입경보(`PERSON_IN_ZONE`), 완료(`TASK_COMPLETE`), 비상정지(`EMERGENCY_STOP`) 실시간 출력.
     - **타워 램프**: 30fps 백그라운드 렌더링 (대기 백색, 주행 주황색 Chase, 수동 녹색, 비상 적-백색 점멸).

---

### 2) 하드웨어 캘리브레이션 실측 툴 (`calibration_test`)
```bash
cd ~/Painter_Robot/build
./calibration_test /dev/serial0
```

* **CLI 주요 명령어**:
  * `move 0.15` : 15cm 전진 실측 테스트
  * `turn 90`  : 90도 회전 실측 테스트
  * `nozzle 1` / `nozzle 0` : 노즐 액추에이터 하강/상승 테스트
  * `status`   : STM32 실시간 스텝수 및 플래그 확인
  * `quit`     : 툴 종료

---

## ⚙️ 시스템 서비스 데몬화 (Systemd Auto-run)

```bash
cd ~/Painter_Robot/daemon
sudo ./install_daemon.sh
```
* **로그 모니터링**:
```bash
journalctl -u robot_exec.service -f -n 50
```
