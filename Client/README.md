# Road Painter Client (Windows Qt 6.11 / QML 관제 클라이언트)

Windows PC 환경에서 구동되는 **Road Painter 중앙 관제 및 노면 도면 작도 소프트웨어**입니다. 한화비전 4채널 CCTV 실시간 스트림 수신, Top View 기반 실치수(mm) 도면 작도, 로봇 실시간 텔레메트리 모니터링, 그리고 TLS 1.2 암호화 제어 명령 송수신을 담당합니다.

---

## 1. 주요 기능 및 아키텍처 (Key Features)

### 1) 실시간 4채널 관제 & H.264 무복사 디코딩
* **독립 스레드 디코딩 (`video_worker`, `preview_worker`):** OpenCV `VideoCapture` (FFmpeg 백엔드)를 통해 MediaMTX RTSP 중계 스트림을 수신하며, 픽셀 버퍼 복사 없는 `QImage` 전달로 2592×1520 고해상도 영상을 67ms 간격으로 부드럽게 렌더링.
* **4분할 및 단일 채널 즉시 전환:** 4채널 동시 미리보기 및 작업 채널 전환 시 지연을 10.5초에서 **4.7초로 55% 단축**.

### 2) Top View 실치수(mm) 도면 작도 엔진 (`VideoView`, `paintgeometry`)
* **호모그래피 기반 실좌표 작도:** $H_{\text{floor}}$ 행렬을 이용해 화면 확대/축소와 무관한 실제 바닥 mm 단위 작도 환경 구현.
* **다양한 형상 지원:** 직선, 사각형, 삼각형, 원(단일 ARC), 텍스트(문자 획), 자유 곡선 지원.
* **실제 도색 완성 치수 환산 (`paintdimensions`):** 60 mm 고정 도포 폭을 고려하여, 사용자가 입력한 수치가 실제 페인트가 칠해진 외곽 끝-끝 치수로 완성되도록 자동 오프셋 보정.

### 3) 펜 접촉 기반 실시간 도색 진행률 시각화 (`paintprogress`, PR #58)
* **172 mm 펜 오프셋 투영:** 로봇 마커 중심이 아닌 실제 펜 끝 좌표를 궤적에 투영하여, 서버의 `STATUS.painting == true` 구간에서만 주황색 도색 완료 선 갱신.
* **오작동 방지:** 펜을 들고 이동하는 접근(Approach) 구간과 꼭짓점 재배치 동작은 진행률 계산에서 제외하여 정확한 0~100% 진행 상태 표시.

### 4) 고신뢰성 보안 통신 & 다단계 안전 인터락 (`ServerClient`, `Backend`)
* **TLS 1.2 인증서 검증:** `QSslSocket` 기반 자체서명 `server.crt` 무결성 검증을 통해 인가되지 않은 외부 접속 차단.
* **다단계 작업 제어:** `START_DRAW` 도색 시작, `ESTOP` 비상정지, `HOLD` 일시정지, `ABORT_DRAW` 작업 취소, 수동 방향키 원격 조작 인터페이스 통합.

---

## 2. 서브시스템 디렉토리 구조

```text
Client/
├── Demo-2/                          ➔ [Qt Creator 프로젝트 소스]
│   ├── CMakeLists.txt               • Qt 6.11 / MinGW 빌드 스크립트
│   ├── qml/                         • QML UI 계층 (MainPage, VideoPane, TopView, ControlPanel)
│   └── src/
│       ├── main.cpp                 • QML 엔진 로드 및 C++ 백엔드 컨텍스트 등록
│       ├── backend (.cpp/.h)        • UI-통신-영상 간 상태 모델 수명주기 총괄
│       ├── serverclient (.cpp/.h)   • TLS 9000 제어 채널 JSON Lines 비동기 송수신
│       ├── videoview (.cpp/.h)      • CCTV / Top View 렌더링, 캘리브레이션 및 작도 시각화
│       ├── video_worker (.cpp/.h)   • 주 화면 RTSP 스트림 디코딩 워커 스레드
│       ├── preview_worker (.cpp/.h) • 4채널 미리보기 스트림 디코딩 워커 스레드
│       ├── paintgeometry (.cpp/.h)  • 60mm 도포 폭 기반 도형 기하학 변환 및 BLUEPRINT 생성
│       ├── paintdimensions (.h)     • 펜 중심선 ↔ 완성 도색 치수 일관 변환기
│       ├── paintprogress (.h)       • 펜 접촉 기반 실시간 도색 진행률 연산기
│       └── robottiming (.h)         • 로봇 정격 속도 기반 작업 예상 소요시간 계산기
├── BUILD.md                         • 의존성 설치 및 릴리즈 빌드 상세 문서
└── RoadPainter.exe                  • 32개 필수 DLL 포함 독립 실행형 배포 패키지 (169 MB)
```

---

## 3. 빌드 및 실행 가이드 (Build & Run)

### 1) 빌드 환경 요구사항
* OS: Windows 10 / 11 (64-bit)
* IDE / 툴체인: Qt Creator 13+, CMake 3.24+, MinGW 13.1 (64-bit), Ninja
* 라이브러리: Qt 6.11.0 (Quick, QuickControls2, Network, Svg), OpenCV 4.12.0

### 2) CMake 릴리즈 빌드
```powershell
cd Client/Demo-2
mkdir build-release ; cd build-release
cmake -G "Ninja" -DCMAKE_BUILD_TYPE=Release -DOpenCV_DIR="C:/opencv/build" ..
ninja
```

### 3) 독립 실행형 배포 패키징 (`windeployqt`)
```powershell
# 배포 폴더로 바이너리 복사 후 의존성 자동 추출
windeployqt --qmldir ../qml RoadPainter.exe
```
* 불필요한 디버그 모듈 및 미사용 번역 파일을 제거하여 **배포 용량을 320 MB에서 169 MB(47% 감소)로 최적화**하였습니다.

---

## 4. 주요 제어 메시지 규격 (JSON Lines)

* **BLUEPRINT (작업 도면 전송):**
  ```json
  {"type":"BLUEPRINT","channel":0,"points":[{"x":1.0,"y":1.0},{"x":2.0,"y":1.0}],"paint":[true],"program":[{"op":"MOVE","dist_m":1.0}]}
  ```
* **START_DRAW / ESTOP / ABORT_DRAW:**
  ```json
  {"type":"CMD","command":"START_DRAW"}
  {"type":"CMD","command":"ESTOP"}
  {"type":"CMD","command":"ABORT_DRAW"}
  ```
