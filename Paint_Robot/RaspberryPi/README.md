# Road-Painter Raspberry Pi Controller

라즈베리파이 4B(로봇 MPU)에서 동작하는 C++ 기반 메인 로봇 제어기 및 네트워크/시리얼 게이트웨이 프로그램입니다. 비전 서버로부터 Wi-Fi(TLS 1.2 소켓)를 통해 **v0.3 프로토콜 명세(`PATH`, `READY`, `GO`, `ALIGN`, `DRIFT`, `CMD`)**를 수신하고, MPU6050 IMU 융합 및 스텝 오도메트리 제어 루프를 통해 STM32 모터 제어기로 UART 속도/노즐 지령을 송신하는 역할을 수행합니다.

---

## 🛠️ 사전 요구사항 및 의존성 설치 (최초 1회)
RPi 내에서 SSL 암호화 소켓 연동, I2C IMU 제어 및 시리얼 통신 빌드를 진행하기 위해 아래 패키지들을 사전에 반드시 설치해야 합니다.

```bash
sudo apt-get update
sudo apt-get install -y g++ make cmake libssl-dev wiringpi
```

---

## 📂 프로젝트 구조

```text
RaspberryPi/
├── CMakeLists.txt         # CMake 빌드 파일
├── README.md              # 본 설명 문서
├── server.crt             # 비전 서버 TLS 연동을 위한 SSL 인증서 (로컬 검증용)
│
├── include/
│   ├── RobotTypes.h       # 공통 구조체, enum 및 프로토콜 규격 정의 헤더
│   ├── SerialManager.h    # RPi ↔ STM32 시리얼(UART) 송수신 클래스
│   ├── NetworkManager.h   # RPi ↔ 비전 서버 TCP/TLS 연결 관리 클래스 (v0.3 명세)
│   ├── PathFollower.h     # 스텝 오도메트리 정밀 주행/회전 및 155mm 노즐 오프셋 제어 클래스
│   ├── ImuManager.h       # MPU6050 I2C 자이로 센서 필터링 및 50Hz 백그라운드 적분 클래스
│   └── nlohmann/          # JSON Parser 라이브러리 디렉토리 (nlohmann/json.hpp)
│
└── src/
    ├── main.cpp           # 메인 시퀀스 오케스트레이터 (v0.3 자율/수동 제어 루프)
    ├── SerialManager.cpp  # UART 포트 개방 및 CRC8 패킷 송수신 구현
    ├── NetworkManager.cpp # TLS 소켓 세션 관리 및 v0.3 JSON Lines 메시지 파싱
    ├── PathFollower.cpp   # 0.1° 회전(22.58 pulse/deg) 및 거리 역산(15433 pulse/m) 수식 구현
    ├── ImuManager.cpp     # MPU6050 I2C DLPF 필터링, 자가 캘리브레이션 및 Yaw 적분 구현
    ├── motor_test.cpp     # STM32 UART 제어 검증 전용 독립 테스트 툴
    └── imu_test.cpp       # MPU6050 I2C 실시간 센서 검증 전용 독립 테스트 툴
```

---

## 🏗️ 빌드 가이드 (Build Guide)

라즈베리파이 터미널(SSH)에 접속한 뒤 아래 명령어를 통해 프로젝트를 빌드합니다.

### 1) 최초 빌드 환경 구성
```bash
cd ~/Painter_Robot
mkdir -p build
cd build
cmake ..
make
```
*   빌드가 완료되면 `build` 폴더 내에 메인 제어 바이너리 **`robot_exec`** 및 테스트 툴 **`motor_test`**가 생성됩니다.

### 2) 소스코드 수정 후 재빌드
소스파일이 변경되었을 때는 `build` 폴더에서 `make`만 수행합니다.
```bash
cd ~/Painter_Robot/build
make
```

---

## 🚀 실행 및 테스트

### 1) 로봇 메인 제어 루프 구동 (`robot_exec`)
시리얼 통신 개방 및 하드웨어 GPIO / I2C 제어 권한 확보를 위해 실행합니다.
```bash
cd ~/Painter_Robot/build
./robot_exec <서버_IP_주소>
```
*   **주요 구동 기능**:
    1.  **초기화**: STM32 부팅 비상정지 래치(`0x16`) 해제 및 MPU6050 자가 캘리브레이션
    2.  **자율 주행**: 서버 `PATH` 수신 시 `0.1°` 회전(2,032 pulse / 90°) 및 지정 거리 정밀 직진 (15,433 pulse / m)
    3.  **노즐 오프셋 제어**: $155\text{ mm}$ ($2,392$ pulse) 물리 오프셋 정렬 시퀀스 지원
    4.  **IMU/DRIFT 융합**: 직진 주행 시 IMU Yaw + 서버 `DRIFT` 피드백 융합 모터 차등 속도(SPS) 조향 제어
    5.  **수동 제어 모드**: 서버 `FORWARD`/`BACKWARD`/`TURN_LEFT`/`TURN_RIGHT`/`STOP`/`NOZZLE_UP`/`NOZZLE_DOWN` 수동 조작 지속 제어

### 2) STM32 UART 송수신 검증 테스트 (`motor_test`)
STM32 간 UART 통신 및 모터/노즐 하드웨어 동작을 독립 검증하기 위한 툴입니다.
```bash
cd ~/Painter_Robot/build
./motor_test
```

---

## ⚙️ 시스템 서비스 데몬화 (Systemd Auto-run)

전원이 켜졌을 때 백그라운드에서 자동으로 제어 프로그램이 켜지게 만들고, 프로그램이 강제 종료되었을 때 자동 재시작이 가능하도록 Systemd 서비스 데몬으로 등록하는 방법입니다.

### 1) 서비스 파일 구성 (`robot_painter.service`)
`/etc/systemd/system/robot_painter.service` 경로에 아래와 같이 구성 파일을 작성합니다.

```ini
[Unit]
Description=Road-Painter Robot Main Controller Service
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=root
WorkingDirectory=/home/user/Painter_Robot
ExecStart=/home/user/Painter_Robot/build/robot_exec
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

### 2) 데몬 명령어 가이드
```bash
# 1. 서비스 파일을 설정 폴더로 복사
sudo cp ~/Painter_Robot/robot_painter.service /etc/systemd/system/

# 2. Systemd 리로드
sudo systemctl daemon-reload

# 3. 부팅 시 자동 실행 등록
sudo systemctl enable robot_painter.service

# 4. 서비스 기동
sudo systemctl start robot_painter.service

# 5. 서비스 정지
sudo systemctl stop robot_painter.service
```

### 3) 실시간 구동 로그 체크
프로그램의 표준 출력을 백그라운드 모니터링합니다.
```bash
journalctl -u robot_painter.service -f -n 50
```
