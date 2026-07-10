# server-rpi

Road-Painter **서버 RPi** 파트. 좌표 변환·경로 계산·중계(서버는 고주기 제어를 하지 않음).

## 폴더 구조

```
server-rpi/
├── include/
│   ├── geometry.hpp     # Pose, Waypoint 등 공용 자료구조
│   └── messages.hpp     # POSE/PATH/STATUS 메시지 (ICD 부록 C)
├── src/
│   ├── main.cpp         # 진입점: TLS 서버 + PATH/POSE 송신 뼈대
│   ├── pose_builder.hpp # ① 좌표계산: 코너px → H_marker → 펜끝 POSE
│   ├── path_builder.hpp # ② 경로생성: 세그먼트 → 웨이포인트(PATH)
│   └── net/
│       └── tls_server.* # OpenSSL TLS 서버 래퍼
├── config/              # 캘리브레이션 yaml (런타임 로드) — CCTV 파트가 생성
├── certs/               # server.crt (+ server.key: 서버만, git 제외)
└── tools/               # 파이썬 캘리브레이션 도구 (설치 전 1회성)
```

## 역할 (SRS 기준)
- 측위 중계: 카메라앱 코너px 수신 → undistort → **H_marker** → 펜끝 POSE → 로봇에 10~15Hz.
- 경로 계산: Qt 작도px → **H_floor** → 획 분해 → PATH → 로봇에 1회.
- 캘리브레이션·결과검사·설정배포·사용자관리.
- ※ 고주기 경로추종 제어는 **로봇 RPi**가 수행(별도 폴더).

## 빌드 & 실행
```bash
sudo apt install libssl-dev cmake g++

# TLS 인증서 준비 (최초 1회)
openssl req -x509 -newkey rsa:2048 -nodes \
  -keyout certs/server.key -out certs/server.crt -days 365 \
  -subj "/CN=roadpainter-server"

cmake -B build && cmake --build build
./build/server_rpi
```
로봇(또는 테스트용 클라이언트)이 `<서버IP>:8443`으로 접속하면 PATH·POSE를 받는다.

## 현재 상태 (뼈대)
- [x] TLS 서버 (openssl s_server 를 코드화)
- [x] PATH/POSE 메시지 송신 구조
- [ ] 측위피드 수신 + undistort (OpenCV 연동)
- [ ] Qt 작도 수신 + H_floor 변환
- [ ] 캘리브레이션 yaml 로드
- [ ] STATUS 수신·처리, 워치독
