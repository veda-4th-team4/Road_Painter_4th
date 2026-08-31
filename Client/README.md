# Road Painter Client (Windows Qt 6.11 / QML 관제 클라이언트)

Windows PC 환경에서 구동되는 **Road Painter 중앙 관제 및 노면 도면 작도 소프트웨어**입니다.

한화비전 4채널 CCTV 실시간 스트림 수신, Top View 기반 실치수(mm) 도면 작도, 로봇 실시간 텔레메트리 모니터링, TLS 암호화 제어 명령 송수신을 담당합니다.

---

## 1. 주요 기능 및 아키텍처

### 1) 실시간 4채널 관제 및 H.264 디코딩

- **독립 스레드 영상 처리 (`video_worker`, `preview_worker`):** 중앙 서버와 연동된 MediaMTX 중계 서버를 통해 4채널 CCTV 영상을 수신하며, 선택 채널과 4채널 미리보기를 각각 독립된 워커 스레드에서 처리합니다.
- **4분할 및 단일 채널 전환:** 4채널 영상을 동시에 미리 보고, 선택한 채널을 작업 화면으로 전환할 수 있습니다. 채널 전환 지연을 10.5초에서 **4.7초로 약 55% 단축**했습니다.

### 2) Top View 실치수(mm) 도면 작도 엔진 (`VideoView`, `paintgeometry`)

- **호모그래피 기반 실좌표 작도:** $H_{\text{floor}}$ 행렬을 이용해 CCTV 영상 좌표를 실제 바닥 좌표로 변환합니다. 화면 확대·축소와 관계없이 밀리미터 단위로 경로를 작성할 수 있습니다.
- **다양한 형상 지원:** 직선, 사각형, 삼각형, 원, 텍스트, 자유 곡선 작도를 지원합니다.
- **실제 도색 완성 치수 환산 (`paintdimensions`):** 60mm 도포 폭을 반영하여 사용자가 입력한 크기와 실제 도색 결과의 외곽 치수가 일치하도록 중심 경로를 자동 보정합니다.

### 3) 펜 접촉 기반 실시간 도색 진행률 시각화 (`paintprogress`)

- **172mm 펜 오프셋 투영:** 로봇 마커 중심이 아닌 실제 펜 끝의 위치를 도색 경로에 투영합니다.
- **도색 구간 판별:** 서버에서 수신한 `STATUS.painting == true` 상태에서만 도색 완료 선을 갱신합니다.
- **오작동 방지:** 펜을 들고 이동하는 접근 구간과 꼭짓점 재배치 구간은 진행률 계산에서 제외합니다.
- **실시간 진행 상태 표시:** 실제 펜의 접촉 위치를 기준으로 도색 완료 구간과 전체 진행률을 0~100%로 표시합니다.

### 4) 보안 통신 및 다단계 작업 제어 (`ServerClient`, `Backend`)

- **TLS 인증서 검증:** `QSslSocket`의 `VerifyPeer` 모드와 내장된 자체서명 `server.crt`를 이용해 접속한 서버의 인증서를 검증합니다.
- **JSON Lines 통신:** 모든 제어 메시지는 TLS 제어 채널을 통해 JSON Lines 형식으로 송수신됩니다.
- **다단계 작업 제어:** `START_DRAW` 도색 시작, `ESTOP` 비상정지, `HOLD` 일시정지, `ABORT_DRAW` 작업 취소를 지원합니다.
- **수동 조작:** 방향키 주행과 노즐 상승·하강을 위한 원격 조작 인터페이스를 제공합니다.

---

## 2. 서브시스템 디렉터리 구조

```text
Client/
├── src/                              ➔ Qt Creator 프로젝트 소스
│   ├── CMakeLists.txt                • Qt 6.11 / MinGW 빌드 설정
│   ├── main.cpp                      • QML 엔진 및 C++ 백엔드 등록
│   ├── MainPage.qml                  • 메인 관제 및 작도 화면
│   ├── ChannelGrid.qml               • 4채널 미리보기 및 채널 선택
│   ├── LoginPage.qml                 • 사용자 로그인 화면
│   ├── backend.cpp / backend.h       • UI·통신·영상 상태 총괄
│   ├── serverclient.cpp / .h         • TLS 제어 채널 및 JSON Lines 통신
│   ├── videoview.cpp / .h            • CCTV·Top View 렌더링 및 작도 시각화
│   ├── video_worker.cpp / .h         • 선택 채널 RTSP 디코딩 워커
│   ├── preview_worker.cpp / .h       • 4채널 미리보기 디코딩 워커
│   ├── paintgeometry.cpp / .h        • 도형 기하 변환 및 BLUEPRINT 생성
│   ├── paintdimensions.h             • 중심선과 완성 도색 치수 변환
│   ├── paintprogress.h               • 도색 진행률 계산
│   ├── routeplan.h                   • 다중 경로 순서 및 이동 구간 계획
│   ├── motionprogram.h               • MOVE·TURN·ARC·NOZZLE 명령 생성
│   ├── robottiming.h                 • 예상 작업 시간 계산
│   ├── certs/server.crt              • TLS 서버 인증서
│   └── tests/
│       └── motionprogram_tests.cpp   • 경로 및 명령 생성 회귀 테스트
├── BUILD.md                          • 빌드 및 배포 상세 문서
├── RoadPainter.exe                   • Windows 실행 파일
└── Qt·OpenCV 런타임 파일             • 독립 실행형 배포 구성
```

---

## 3. 빌드 및 실행 가이드

### 1) 빌드 환경 요구사항

- **운영체제:** Windows 10 / 11 64-bit
- **Qt:** Qt 6.11.0 MinGW 64-bit
- **컴파일러:** MinGW 13.1 64-bit
- **빌드 도구:** CMake 3.24 이상, Ninja
- **라이브러리:** OpenCV 4.12.0 MinGW 빌드
- **Qt 모듈:** Quick, QuickControls2, Network, Svg, Gui

### 2) 바로 실행하기

빌드가 필요하지 않은 경우 저장소를 받은 후 다음 파일을 실행합니다.

```text
Client/RoadPainter.exe
```

Qt와 OpenCV 런타임 파일이 `Client/` 폴더에 함께 포함되어 있으므로 별도 설치 없이 실행할 수 있습니다.

### 3) CMake 릴리즈 빌드

PowerShell에서 저장소 루트를 기준으로 실행합니다.

```powershell
cd Client

cmake -S src -B build -G "Ninja" `
  -DCMAKE_BUILD_TYPE=Release `
  -DOpenCV_DIR="C:/opencv/mingw"

cmake --build build
```

빌드 결과는 다음 경로에 생성됩니다.

```text
Client/build/dist/RoadPainter.exe
```

### 4) 테스트 실행

경로 및 로봇 명령 생성 회귀 테스트를 포함해 빌드하려면 다음 옵션을 사용합니다.

```powershell
cd Client

cmake -S src -B build-test -G "Ninja" `
  -DCMAKE_BUILD_TYPE=Release `
  -DOpenCV_DIR="C:/opencv/mingw" `
  -DROADPAINTER_BUILD_TESTS=ON

cmake --build build-test
ctest --test-dir build-test --output-on-failure
```

### 5) 독립 실행형 배포 패키징 (`windeployqt`)

```powershell
C:\Qt\6.11.0\mingw_64\bin\windeployqt.exe `
  --qmldir Client\src `
  Client\build\dist\RoadPainter.exe
```

OpenCV DLL은 CMake 빌드 과정에서 `dist` 폴더로 복사됩니다. 배포 전에는 Qt가 설치되지 않은 Windows PC에서 실행하여 누락된 DLL이 없는지 확인해야 합니다.

불필요한 디버그 모듈과 미사용 런타임 파일을 제거해 배포 용량을 약 **320MB에서 169MB로 축소**했습니다.

---

## 4. 주요 제어 메시지 규격

클라이언트와 중앙 서버는 TLS 제어 채널에서 **JSON Lines** 형식으로 통신합니다.

모든 메시지는 `type`, `seq`, `payload` 구조를 사용하며 JSON 객체 하나가 한 줄을 구성합니다.

### 1) BLUEPRINT

작도 경로와 도색 여부, 로봇 동작 명령을 서버에 전송합니다.

```json
{
  "type": "BLUEPRINT",
  "seq": 1,
  "payload": {
    "points": [
      [1.0, 1.0],
      [2.0, 1.0]
    ],
    "paint": [
      true,
      true
    ],
    "program": [
      {
        "op": "MOVE",
        "v": 0,
        "heading_deg": 0.0,
        "dist_m": 1.0,
        "paint": true
      }
    ]
  }
}
```

- `points`: 실제 바닥 기준 미터 좌표
- `paint`: 각 경로 지점의 도색 여부
- `program`: `MOVE`, `TURN`, `ARC`, `NOZZLE`로 구성된 동작 시퀀스

### 2) START_DRAW

```json
{
  "type": "CMD",
  "seq": 2,
  "payload": {
    "cmd": "START_DRAW"
  }
}
```

### 3) ESTOP

```json
{
  "type": "CMD",
  "seq": 3,
  "payload": {
    "cmd": "ESTOP"
  }
}
```

### 4) ABORT_DRAW

```json
{
  "type": "CMD",
  "seq": 4,
  "payload": {
    "cmd": "ABORT_DRAW"
  }
}
```

- `ESTOP`: 현재 동작을 즉시 정지하며 실행 중인 경로는 유지합니다.
- `ABORT_DRAW`: 서버와 로봇에 저장된 실행 경로를 폐기하고 작업을 종료합니다.

---

## 5. 관련 문서

- [`BUILD.md`](BUILD.md): 상세 빌드 및 배포 절차
- [`PNM_4CH_QT_PLAN.md`](PNM_4CH_QT_PLAN.md): 한화비전 4채널 CCTV 연동 설계
- [`../Server/docs/PROTOCOL.md`](../Server/docs/PROTOCOL.md): 중앙 서버 통신 프로토콜
