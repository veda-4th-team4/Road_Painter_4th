
<p align="center">
  <img src="https://img.shields.io/badge/C++-17-00599C?style=flat&logo=c%2B%2B" />
  <img src="https://img.shields.io/badge/Qt-6.11-41CD52?style=flat&logo=qt" />
  <img src="https://img.shields.io/badge/STM32-FreeRTOS-03234B?style=flat&logo=stmicroelectronics" />
  <img src="https://img.shields.io/badge/Linux-Kernel_LKM-FCC624?style=flat&logo=linux" />
  <img src="https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?style=flat&logo=opencv" />
  <img src="https://img.shields.io/badge/Platform-Hanwha_Vision_OpenSDK-EA1D2C" />
</p>

# Road Painter (고정형 CCTV 기반 경로 작도·측위 연동 노면 도장 로봇)

> **"기존 CCTV로 바닥 작업을 자동화하다"**  
> **한화비전 VEDA 부트캠프 4기 최종 프로젝트 (8회차 4팀)**  
> **개발 기간:** 2026. 07. 02. ~ 2026. 08. 26. (약 8주)

---

## [ 프로젝트 공식 문서 & 아카이브 ]
* **Team Confluence Wiki:** [[https://eosang60.atlassian.net/wiki/...](https://eosang60.atlassian.net/wiki/x/tQAR)]
* **Jira Project Management:** [https://eosang60.atlassian.net/jira/software/projects/SCRUM/boards/1?filter=&groupBy=none&atlOrigin=eyJpIjoiOGY0MDE4OTY5ZDhlNGIyODk3ZGJlMTlkNWY4NWYzYjQiLCJwIjoiaiJ9]
* **시연 영상 및 발표 자료:** [시연 영상 및 발표 PPTX 링크]

---

## 1. 프로젝트 개요 (Overview)

대형 주차장, 물류센터, 공장 등에서 반복적으로 시공되는 주차선·안전선·안내 도형 도색 작업은 기존에 사람이 유해 화학 도료를 직접 도포하는 고비용·고위험 수작업에 의존해 왔습니다.

**Road Painter**는 천장에 이미 설치된 **한화비전 4채널 CCTV 인프라를 외부 측위 및 안전 감시 센서로 재사용**하고, 관제 PC(Qt)에서 작성한 도면을 **자체 제작한 6WD 노면 도장 로봇**이 밀리미터(mm)급 정밀도로 바닥에 자율 도색하는 **End-to-End 영상보안-로보틱스 통합 솔루션**입니다.

### 핵심 차별점
1. **도입 비용 절감 :** 7천만~1.2억 원에 달하는 고가 RTK/토털스테이션 대신 기설치된 CCTV를 활용하여 **추가 하드웨어 20만 원 수준**으로 시스템 구축
2. **온카메라 엣지 AI & 안전 감시:** OpenSDK 기반 Raw ArUco 외부 측위(196.4 ms 지연)와 WiseAI 사람 감지 및 작업구역 침입 경보(I2S 음성 안내 & 자동 HOLD 정지) 연동
3. **초정밀 경로 추종 (1~2 mm 오차):** 155 mm 펜 오프셋 보정, 원호(Arc) 진입 위상 보정, `ALIGN`·`MORE`·`DRIFT` 폐루프 제어로 **원호 이탈 1 mm 이내, 사각형 꼭짓점 오차 1~2 mm 달성**
4. **리눅스 커널 모듈(LKM) + STM32 20kHz 하드 리얼타임 이중 제어기:** OS 지연 없는 I2S 오디오·PWM-DMA LED·500Hz IMU 커널 드라이버와 20 kHz 하드웨어 인터럽트 기반 모터 구동

---

## 2. 전체 시스템 아키텍처 (System Architecture)

<img width="689" height="469" alt="image" src="https://github.com/user-attachments/assets/f64837f0-4aa8-4328-bed4-0434f6369171" />


---

## 3. 서브시스템별 저장소 구조

| 디렉토리 (Module) | 담당 역할 및 기술 스택 | 세부 문서 링크 |
| :--- | :--- | :--- |
| **`CCTV_4ch_wiseai/`** | 한화비전 4채널 OpenSDK 카메라 앱 (C++, OpenCV, Dynamic ROI, WiseAI Metadata) | [CCTV README](CCTV_4ch_wiseai/README.md) |
| **`Server/`** | 중앙 관제 라우터 & MediaMTX RTSP 중계 (C++17, TLS 1.2, nlohmann/json) | [Server README](Server/README.md) |
| **`Client/`** | Windows 관제 클라이언트 UI (Qt 6.11, QML, OpenCV, Top View 작도 엔진) | [Client README](Client/README.md) |
| **`Paint_Robot/`** | 로봇 MPU(RPi 4B LKM/C++) 및 MCU(STM32 FreeRTOS 20kHz IRQ) 펌웨어 | [Robot README](Paint_Robot/README.md) |

---

## 4. 핵심 검증 및 정량 성능 지표 (Key Results)

| 평가 항목 (Metric) | 초기치 / 상용 제품 | Road Painter 최종 결과 | 개선 및 성능 의미 |
| :--- | :---: | :---: | :--- |
| **원호(Arc) 궤적 이탈** | $310\text{ mm}$ ($0.31\text{ m}$) | **$1\text{ mm}$ 이내** | 펜 오프셋 진입 위상 보정으로 폐루프 진원 완성 |
| **사각형 꼭짓점 오차** | $6 \sim 8\text{ mm}$ | **$1 \sim 2\text{ mm}$** | ALIGN 1.0°/MORE 5mm 피드백으로 정밀 안착 |
| **카메라 엣지 측위 지연** | $499.8\text{ ms}$ (RTSP) | **$196.4\text{ ms}$ (Raw)** | $304\text{ ms}$ 단축 (초당 5 fps 실시간 제어 확보) |
| **Dynamic ROI 처리율** | $2.00\text{ fps}$ (CPU 69.5%) | **$10.12\text{ fps}$ (CPU 40.2%)** | 4채널 검출 시간 $278\text{ms} \rightarrow 5.1\text{ms}$ 대폭 단축 |
| **회전 정렬(ALIGN) 횟수**| 최대 6회 진동 | **$1 \sim 2\text{회}$ 수렴** | IMU yaw 연동 및 회전 감속으로 정렬 안정화 |

---

## 5. 빠른 시작 가이드 (Quick Start Guide)

### 1) 중앙 관제 서버 실행 (`Server`)
```bash
# 라즈베리파이 서버 환경에서 실행
cd Server
make -j4
./server 9000
```

### 2) 관제 클라이언트 실행 (`Client`)
```bash
# Windows 환경 (Qt 6.11 / MinGW)
cd Client/Demo-2/build-release
./appRoadPainter.exe
```

### 3) 로봇 제어기 실행 (`Paint_Robot`)
```bash
# 라즈베리파이 로봇 MPU (192.168.0.4)
cd ~/Painter_Robot/build
./robot_exec 192.168.0.8    # 관제 서버 IP 지정
```

---

## 6. 팀원 소개 및 역할 분담 (Team Members & Roles)

| 성명 | 역할 | 주도 업무 |
| :--- | :--- | :--- |
| **어형진** | **팀장** / Qt·QML·STM32·PCB | 로그인·TLS FrameReceiver, 4채널 미리보기·채널 전환, Top View·도형 작도, 캘리브레이션, 수동 제어, 작업 실행·정지, 배포·발표 UART/STM32 모터 제어, 로봇 PCB 설계 |
| **김혁** | 로봇 · HW | 6WD 기구 조립 및 역기구학 및 경로 추종, 감속 주행 및 궤적 제어, 직진 · 회전 캘리브레이션 튜닝 및 IMU 센서 연동, 노즐 서보 모터 안착 제어 및 주행 검증 |
| **심규영** | CCTV / OpenSDK | 한화비전 4채널 CCTV 엣지(카메라 내장) 앱 개발: ArUco·Dynamic ROI, 캘리브레이션·호모그래피 좌표 변환, WiseAI Metadata/IVA·근접 판단, 서버 연동(TCP/TLS), 지연 측정 |
| **염윤서** | 중앙 서버 · LED | TLS 라우팅, 좌표·경로 변환, v2 프로토콜, ALIGN·MORE·DRIFT·HOLD 실시간 피드백, WS2812B 커널 드라이버·LedStripManager |
| **박동진** | **멘토** (한화비전) | 주제 현실성, 오차 지표, LDC·캘리브레이션, 지연·fps 측정, 감속·안전·완성도 피드백 |
