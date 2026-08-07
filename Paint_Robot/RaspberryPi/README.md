# Road-Painter Raspberry Pi Controller (Protocol v2)

라즈베리파이 4B(로봇 MPU)에서 동작하는 C++ 기반 메인 로봇 제어기 및 네트워크/시리얼 게이트웨이 프로그램입니다. 비전 서버로부터 Wi-Fi(TLS 1.2 소켓)를 통해 **Server-Master Protocol v2 (`ops` 배열, `op_index` 트랜잭션 매칭, `READY`, `GO`, `ALIGN`, `MORE`, `DRIFT`, `HOLD`, `PATH_DONE`)**를 수신하고, 스텝 오도메트리 제어 루프를 통해 STM32 모터 제어기로 UART 속도/노즐 지령을 송신하는 역할을 수행합니다.

---

## 🛠️ 사전 요구사항 및 의존성 설치 (최초 1회)

RPi 내에서 SSL 암호화 소켓 연동, I2C IMU 제어 및 시리얼 통신 빌드를 진행하기 위해 아래 패키지들을 사전에 설치합니다.

```bash
sudo apt-get update
sudo apt-get install -y g++ make cmake libssl-dev wiringpi
```

---

## 📂 프로젝트 구조

```text
RaspberryPi/
├── CMakeLists.txt         # CMake 빌드 설정 파일
├── README.md              # 본 설명 문서
│
├── include/
│   ├── RobotTypes.h       # 공통 구조체, enum, Protocol v2 payload 및 ms 타임스탬프 헬퍼
│   ├── SerialManager.h    # RPi ↔ STM32 시리얼(UART 115200bps) 송수신 클래스
│   ├── NetworkManager.h   # RPi ↔ 비전 서버 TCP/TLS 연결 및 Protocol v2 JSON 파서 클래스
│   ├── PathFollower.h     # 스텝 오도메트리 정밀 주행/회전 및 DRIFT 연속 차동 조향 클래스
│   ├── ImuManager.h       # MPU6050 I2C 자이로 센서 필터링 및 적분 클래스
│   └── nlohmann/          # JSON Parser 라이브러리 디렉토리 (nlohmann/json.hpp)
│
└── src/
    ├── main.cpp           # 메인 시퀀스 오케스트레이터 (Protocol v2 핸드셰이크 & 제어 루프)
    ├── SerialManager.cpp  # UART 포트(/dev/serial0) 개방 및 CRC8 패킷 송수신 구현
    ├── NetworkManager.cpp # TLS 소켓 세션 관리, op_index 매칭 및 HOLD 처리 구현
    ├── PathFollower.cpp   # 기구학 스텝 환산 및 후진/직진/턴 조향 알고리즘 구현
    ├── ImuManager.cpp     # MPU6050 I2C DLPF 필터링 및 Yaw 처리 구현
    ├── calibration_test.cpp# 🎯 하드웨어 캘리브레이션 (거리/각도 실측 보정) 대화형 CLI 툴
    ├── motor_test.cpp     # STM32 UART 모터/노즐 하드웨어 단독 검증 툴
    ├── arc_test.cpp       # 호(Arc) 주행 기구학 검증 툴
    └── imu_test.cpp       # MPU6050 I2C 실시간 센서 검증 툴
```

---

## 📐 하드웨어 기구학 규격 & 보정 계수

* **바퀴 지름 (Wheel Diameter)**: $66\text{ mm}$ ($0.066\text{ m}$)
* **차축 거리 (Wheelbase)**: $166\text{ mm}$ ($0.166\text{ m}$)
* **1회전당 스텝 수**: $3,200\text{ steps/rev}$ (16 마이크로스텝)
* **거리 보정 계수 (`DISTANCE_CALIB_FACTOR`)**: `1.010f` (+1% 실측 보정) $\rightarrow$ 약 $15,587\text{ steps/m}$ ($10\text{cm} = \mathbf{1,559\text{ 스텝}}$)
* **회전 보정 계수 (`TURN_ANGLE_CALIB_FACTOR`)**: `1.030f` (+3% 실측 보정) $\rightarrow$ 약 $2,073\text{ steps/90}^\circ$ ($1^\circ = \mathbf{23.03\text{ 스텝}}$)

---

## 🏗️ 빌드 가이드 (Build Guide)

라즈베리파이 터미널(SSH)에 접속한 뒤 아래 명령어로 빌드합니다.

### 1) 빌드 및 바이너리 생성
```bash
cd ~/Painter_Robot
mkdir -p build
cd build
cmake ..
make
```
* 빌드가 완료되면 `build` 폴더 내에 메인 제어 바이너리 **`robot_exec`** 및 캘리브레이션 툴 **`calibration_test`**가 생성됩니다.

---

## 🚀 실행 및 테스트 가이드

### 1) 로봇 메인 제어 루프 구동 (`robot_exec`)
```bash
cd ~/Painter_Robot/build
./robot_exec
```

* **Protocol v2 제어 특징**:
  1. **`op_index` 트랜잭션 검증**: 수신되는 모든 `GO`, `ALIGN`, `MORE`, `DRIFT` 명령의 `op_index`를 엄격 대조하여 지연 응답에 의한 오작동 완전 차단.
  2. **`ALIGN` / `MORE` 2초 안착 딜레이**: 미세 회전 및 미세 전진/후진 완료 후 차체/카메라 진동 방지를 위해 2.0초(2000ms) 강제 안착 대기 후 서버로 `READY` 전송.
  3. **`DRIFT` 주행 중 연속 차동 조향**: 직진 도색 주행 중 멈추지 않고 실시간으로 좌/우 바퀴 차동 속도(`L: 875, R: 667`)를 제어하여 보정 조향.
  4. **`HOLD` 자동 일시정지 및 재개**: 비전 위치 두절($\ge 2$초) 시 `HOLD: PAUSE` 수신으로 즉시 정지, 회복 시 `HOLD: RESUME`로 멈췄던 자리에서 주행 자동 재개.
  5. **밀리초 정밀 타임스탬프**: 모든 콘솔 출력 로그에 `[HH:MM:SS.mmm]` 타임스탬프를 부여하여 비전 서버 로그와 1:1 시각 동기화 트래킹.

---

### 2) 하드웨어 캘리브레이션 실측 툴 (`calibration_test`)
자 단위 / 각도기 단위로 실제 이동 거리와 회전 각도를 검증하는 대화형 터미널 툴입니다.

```bash
cd ~/Painter_Robot/build
./calibration_test
```

* **CLI 주요 명령어**:
  * `move 0.1` : 10cm 전진 실측 테스트 (`move -0.1` 은 10cm 후진)
  * `turn 20`  : 20도 우회전(CW) 실측 테스트 (`turn -20` 은 20도 좌회전)
  * `nozzle 1` / `nozzle 0` : 노즐 액추에이터 하강/상승 테스트
  * `status`   : STM32 실시간 엔코더 및 플래그 상태 출력
  * `quit`     : 툴 종료

---

## ⚙️ 시스템 서비스 데몬화 (Systemd Auto-run)

전원이 켜졌을 때 백그라운드에서 자동으로 제어 프로그램이 켜지도록 등록하는 방법입니다.

### 1) 서비스 파일 복사 및 등록
```bash
# 1. Systemd 서비스 파일 복사 및 리로드
sudo cp ~/Painter_Robot/robot_painter.service /etc/systemd/system/
sudo systemctl daemon-reload

# 2. 부팅 시 자동 실행 등록 및 서비스 기동
sudo systemctl enable robot_painter.service
sudo systemctl start robot_painter.service
```

### 2) 실시간 구동 로그 체크
```bash
journalctl -u robot_painter.service -f -n 50
```
