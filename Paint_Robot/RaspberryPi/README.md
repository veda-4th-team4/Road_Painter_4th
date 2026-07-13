# Road-Painter Raspberry Pi Controller

라즈베리파이 4B(로봇 MPU)에서 동작하는 C++ 기반 메인 로봇 제어기 및 네트워크/시리얼 게이트웨이 프로그램입니다. 비전 서버로부터 Wi-Fi를 통해 절대 위치(`POSE`)를 수신하고, 경로 추종 제어 루프를 거쳐 STM32 모터 제어기로 UART 속도 지령을 송신하는 역할을 수행합니다.

---

## 🛠️ 사전 요구사항 및 의존성 설치 (최초 1회)
RPi 내에서 SSL 암호화 소켓 연동 및 빌드를 진행하기 위해 아래 패키지들을 사전에 반드시 설치해야 합니다.

```bash
sudo apt-get update
sudo apt-get install -y g++ make libssl-dev
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
│   ├── NetworkManager.h   # RPi ↔ 비전 서버 TCP/TLS 연결 관리 클래스 (뼈대)
│   ├── PathFollower.h     # 경로 추종 유도 제어 및 차동 구동 기구학 변환 클래스 (뼈대)
│   └── nlohmann/          # JSON Parser 라이브러리 디렉토리
│
└── src/
    ├── main.cpp           # 전체 모듈 기동 및 통신 연동 루프 오케스트레이터
    ├── robot_sim.cpp      # TCP/TLS 통신 테스트용 시뮬레이션 클라이언트
    ├── SerialManager.cpp  # wiringPi 기반 시리얼 포트 개방 및 패킹 전송 구현
    ├── NetworkManager.cpp # 소켓 연결 및 수신 파서 스텁 구현
    └── PathFollower.cpp   # 선속도/각속도 sps 변환 기구학 수식 구현
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
*   빌드가 완료되면 `build` 폴더 내에 두 개의 바이너리 **`robot_exec`** 및 **`robot_sim`**이 생성됩니다.

### 2) 소스코드 수정 후 재빌드
소스파일이 변경되었을 때는 `build` 폴더에서 `make`만 수행합니다.
```bash
cd ~/Painter_Robot/build
make
```

---

## 🚀 실행 및 테스트

### 1) 로봇 메인 제어 루프 구동 (`robot_exec`)
시리얼 통신 개방 및 하드웨어 GPIO 핀 제어 권한 확보를 위해 **`sudo`** 권한으로 실행합니다.
```bash
cd ~/Painter_Robot/build
sudo ./robot_exec
```
*   현재 `main.cpp`는 STM32 통신 검증용 루프가 가동 중입니다:
    1.  좌우 스텝모터 sps 제어 지령 송신 (`500sps`, `-500sps`)
    2.  페인팅 노즐 ON 지령 송신
    3.  비상정지(E-Stop) 지령 송신

### 2) TCP/TLS 통신 시뮬레이션 테스트 (`robot_sim`)
비전 서버와의 TLS 소켓 통신을 검증하기 위한 가상 테스트 클라이언트입니다.
```bash
cd ~/Painter_Robot/build
# 사용법: ./robot_sim <서버_IP_주소>
./robot_sim 192.168.0.8
```
*   정상 연결 시 서버로 1초마다 로봇의 `STATUS` 패킷을 주기 송출합니다.

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
