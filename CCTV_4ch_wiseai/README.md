# CCTV 4ch WiseAI (한화비전 4채널 OpenSDK 온카메라 엣지 비전 앱)

한화비전 4채널 멀티디렉셔널 AI 카메라(**PNM-C16083RVQ**, Wisenet Open Platform)의 온카메라(On-Camera) 임베디드 리눅스 환경에서 구동되는 C++ 기반 엣지 비전 및 작업자 안전 감시 애플리케이션입니다.

카메라 내부 ISP로부터 **NV12 Raw 영상(Y-Plane)**을 직접 획득하여 실시간 ArUco 외부 측위를 수행하고, **WiseAI Human Metadata**를 연동하여 작업 구역 내 작업자 침입 및 사람-로봇 간 안전 거리를 판정합니다.

---

## 1. 주요 기능 및 아키텍처 (Key Features)

### 1) 초저지연 Raw 영상 ArUco 외부 측위 (`ArucoProcessor`)
* **H.264 인코딩 지연 우회:** RTSP 스트림(지연 499.8 ms) 대신 온카메라 Raw 메모리(`SPMgrVideoRaw`)에서 직접 Y-Plane 그레이스케일을 추출하여 **전송 지연을 196.4 ms로 대폭 단축 (304 ms 개선)**.
* **4채널 독립 좌표계 처리:** 4개 렌즈의 K/D(왜곡계수)와 Homography를 채널별로 완전 분리 관리하여 채널 간 좌표 간섭 원천 차단.

### 2) Dynamic ROI 적응형 관심영역 추적 (`DynRoiTracker`)
* **검출 부하 최적화:** 4채널 전체 화면을 매 프레임 탐색하지 않고, 검출된 마커 주변 240px ROI만 집중 추적(`TRACK` 모드).
* **단계적 확대 재시도:** 미검출 시 50%씩 4단계 확대 후 전체 탐색(`SEARCH` 모드)으로 복귀.
* **정량 성과:** 4채널 동시 구동 시 CPU 점유율 **69.5% ➔ 40.2%** 감소, 실효 처리율 **2.00 fps ➔ 10.12 fps (5.1배 향상)**, 검출 시간 **278 ms ➔ 5.1 ms** 단축.

### 3) 바닥 및 마커 평면 시차 분리 호모그래피 (`HomographyMapper`)
* **$H_{\text{floor}}$ 정적 앵커 피팅:** 바닥 실측 앵커 좌표 수집(20프레임 평균) 및 Leave-One-Out(LOO) 잔차 자동 검증.
* **$H_{\text{marker}}$ 시차(Parallax) 보정:** 카메라 시점과 마커 높이(130mm)에 따른 투영 밀림 오차를 기하학적 법선 벡터로 보정.
* **오도메트리 사각 주행 캘리브레이션:** 바닥 앵커 없이 로봇 주행 궤적만으로 $H_{\text{marker}}$를 자동 추출하는 확장 모드 지원.

### 4) WiseAI 온카메라 작업자 안전 감시 (`WiseAiMetadata`, `ProximityGuard`)
* **발끝 좌표 기반 존 판정 (`ZONE_EVENT`):** WiseAI Human BBox 하단 중앙점을 $H_{\text{floor}}$로 바닥 좌표 변환하여 작업 영역 침입을 밀리미터 단위로 정밀 판정 (주의 500 mm / 위험 300 mm).
* **사람-로봇 실시간 근접 경보:** 사람과 로봇 간 2D 유클리드 거리를 계산하여 `SAFE` / `CAUTION` / `DANGER` 3단계 상태를 중앙 서버 및 로봇으로 즉시 통보.

---

## 2. 서브시스템 디렉토리 구조

```text
CCTV_4ch_wiseai/
├── app/
│   ├── src/ & include/
│   │   ├── SampleComponent (.c/.h)   • OpenSDK 이벤트 루프 및 전체 모듈 수명주기 관리
│   │   ├── ArucoProcessor (.cpp/.h)   • OpenCV ArUco 검출 및 Grayscale Y-Plane 처리
│   │   ├── DynRoiTracker (.cpp/.h)    • TRACK / SEARCH 상태 머신 및 동적 ROI 계산
│   │   ├── HomographyMapper (.cpp/.h) • K/D 렌즈 보정, H_floor 및 H_marker 변환/LOO 검증
│   │   ├── IntrinsicsCalib (.cpp/.h)  • ChArUco 기반 4채널 내부 파라미터 캘리브레이션
│   │   ├── WiseAiMetadata (.cpp/.h)   • ONVIF Metadata 파싱 및 Human BBox 발끝 좌표 추출
│   │   ├── ProximityGuard (.cpp/.h)   • 사람-로봇 300ms 디바운스 근접 거리 판정기
│   │   ├── central_tls_sender (.cpp)  • 중앙 서버 TLS 1.2 소켓 송수신 클라이언트
│   │   └── pose_sender (.cpp)         • 로봇 MPU 직결 Non-blocking TCP 송신 모듈
│   ├── third_party/                   • OpenCV 4.x aarch64 타겟 정적 라이브러리
│   └── toolchain.cmake                • ARM Cortex-A53 aarch64 크로스 컴파일러 설정
├── config/                            • 채널별 K, D, H_floor, H_marker JSON 설정 저장소
├── build_install.sh                   • 도커 기반 크로스 컴파일 및 카메라 자동 배포 스크립트
└── docker-compose.yml                 • OpenSDK aarch64 빌드 환경 컨테이너 설정
```

---

## 3. 빌드 및 카메라 배포 가이드 (Build & Deploy)

### 1) Docker 기반 크로스 컴파일 및 패키징
```bash
# 호스트 PC (Linux / WSL2)
cd CCTV_4ch_wiseai
docker-compose up -d
docker exec -it cctv_builder bash -c "./build_install.sh build"
```
* 빌드 완료 시 `app/build/CCTV_4ch_wiseai.cap` OpenSDK 앱 패키지 파일이 생성됩니다.

### 2) 카메라 설치 및 환경 설정 (`camera.env`)
```bash
# 카메라 IP 및 계정 설정 후 원격 설치
cp camera.env.example camera.env
# camera.env 내 CAMERA_IP, ADMIN_ID, ADMIN_PW 기입
./build_install.sh install
```

---

## 4. 주요 통신 규격 (Output JSON)

* **CAM_POSE (서버 전송):**
  ```json
  {"type":"CAM_POSE","channel":0,"marker_id":42,"x":1.245,"y":0.832,"theta":-45.2,"timestamp":1724200000123}
  ```
* **ZONE_EVENT (위험 침입):**
  ```json
  {"type":"ZONE_EVENT","channel":0,"event":"Enter","distance_mm":240,"status":"DANGER"}
  ```
