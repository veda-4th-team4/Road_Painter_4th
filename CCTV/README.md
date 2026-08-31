# Road Painter — CCTV 캘리브레이션 & 호모그래피 검증 앱 (`cctv_app`)

본 디렉터리는 **Road Painter**의 초기 단일 렌즈 측위 모듈, 즉 한화비전 카메라
내부에서 구동되는 **온카메라 관리자·시운전용 앱(`.cap`)** 소스와 크로스 빌드
스크립트를 포함합니다. ArUco 마커를 검출해 카메라 내부 파라미터(K/dist)를 보정하고,
픽셀→월드 **호모그래피를 카메라 위에서 직접 계산·검증**하는 것이 목적입니다.

> 후속 4채널 버전은 [`CCTV_4ch_wiseai/`](../CCTV_4ch_wiseai/README.md)이며,
> 이 앱에서 검증된 호모그래피·캘리브레이션 로직을 이식해 사용합니다.

---

##  1. 핵심 사양 & 기준값

### 1) 시스템 사양

| 항목 (Specification) | 수치 / 규격 | 세부 비고 |
| :--- | :--- | :--- |
| **대상 카메라** | **PNO-A9081RG** (단일 렌즈) | `IPCameraManifest.xml`의 `<model>` 선언값 |
| **빌드 플랫폼** | **Hanwha OpenSDK 5.00** | 매니페스트 `min/target/maxSDK` 모두 `5.00` |
| **빌드 타깃** | **`bin/cctv_app_cv2x`** (Ambarella CV2x) | `Makefile`의 `all:` 기본 타깃, 총 6개 SoC 정의 |
| **영상 입력** | **Raw NV12 $1920\times1080$ @ 10 fps** | 인코딩 스트림은 H.264 30 fps (별개 경로) |
| **마커 규격** | **`DICT_4X4_50`**, 한 변 $50\text{ mm}$ | `MARKER_LENGTH_M 0.05f`, 로봇 마커 ID `49` |
| **좌표 변환** | **픽셀→월드 호모그래피**, `raw` / `undistort` | 좌표 모드는 H와 함께 메타데이터로 저장 |
| **내부 파라미터** | **ChArUco 기반 K/dist 캘리브레이션** | 캡처 품질 게이트 + LDC 실시간 검사 |
| **관리자 대시보드** | TCP `7000`(pose·명령) / `7001`(스냅샷) | 브라우저 UI는 `8083`, `Server/admin_console/` |
| **중앙 서버 링크** | **TLS `9000`** (`role=CCTV`, legacy) | 서버 인증, 비차단 재연결 |
| **런타임 명령** | **54종** | 재빌드 없이 대시보드에서 투입 |

### 2) 호모그래피 기준 마커

컴파일 기본값입니다. 앵커·검증 목록과 월드 좌표는 대시보드에서 런타임 변경할 수
있으며(`ANCHOR_SET`, `VALIDATION_SET`), 현장 설치 전 마커 **중심**의 실측 mm
좌표로 교체해야 합니다.

| 구분 | 기본 ID | 개수 | 역할 |
| :--- | :---: | :---: | :--- |
| **계산 앵커** | `0` ~ `7` | 8 | 8개가 **모두 보이는** 프레임 30장을 모아 H 계산 |
| **검증 마커 (내부)** | `13` ~ `17` | 5 | 사각형 내부 — 보간 정확도 검증 (중앙 1 + 사분면 4) |
| **검증 마커 (외부)** | `9` ~ `12` | 4 | 각 변 중점에서 약 $300\text{ mm}$ 바깥 — 외삽 검증 |

기본 앵커 배치는 $3000 \times 2000\text{ mm}$ 직사각형의 네 꼭짓점·네 변 중점입니다.
검증 마커는 **어떤 모드에서도 H 피팅에 포함되지 않으며**, H로 환산한 위치와 실측
좌표를 비교해 마커별 오차·RMSE·최대 오차를 산출합니다.

수집 조건은 `CALIB_GOOD_FRAMES 30` / `CALIB_MAX_FRAMES 300`입니다 — 유효 프레임
30장을 모으면 계산하고, 총 300 프레임(5 fps 기준 약 60초) 안에 못 모으면 포기합니다.

### 3) 좌표 모드 규칙

H를 **만들 때와 적용할 때의 픽셀 좌표 모드는 반드시 같아야 합니다.**

* `raw` — 검출된 원본 픽셀 코너를 그대로 사용
* `undistort` — K/dist로 보정한 픽셀 좌표를 사용
* 마커 높이 평면용 `H_marker` 유도는 핀홀 모델을 전제하므로 **`undistort` 모드에서만**
  허용됩니다

---

##  2. 시스템 아키텍처 및 디렉터리 구조

### 1) 데이터 흐름

```text
PNO-A9081RG / cctv_app.cap
  ├─ Raw NV12 1920×1080 ─▶ 동적 ROI ─▶ ArUco 검출 ─▶ 호모그래피 환산
  ├─ TCP  CAM_POSE / 명령 ───────────▶ 관리자 대시보드 :7000
  ├─ TCP  LDC_SNAPSHOT ──────────────▶ 관리자 대시보드 :7001
  └─ TLS  HELLO / POS / H_MATRIX ────▶ 중앙 관제 서버 :9000

브라우저 ◀── :8083 ── 관리자 대시보드 (Server/admin_console/web_gui.py)
```

카메라는 컴파일 시점에 정해진 **한 포트로만** 접속하므로(`POSE_SERVER_PORT`),
대시보드 인스턴스가 여러 개여도 카메라를 받는 쪽은 항상 하나입니다.

### 2) 디렉터리 구조

```text
CCTV/
├── src/                              ➔ [앱 본체]
│   ├── aruco_detector_cv.cpp         • main·프레임 루프·명령 디스패치·CAM_POSE 조립
│   ├── aruco_processor.cpp           • ArUco 검출기 + 포즈 추정 (OpenSDK 래핑)
│   ├── dyn_roi.cpp                   • 마커를 따라가는 적응형 ROI (픽셀 수 = 검출 비용)
│   ├── detect_tuning.cpp             • 검출 성능 설정 영속화 (탐색 ROI + 적응 임계 패스 수)
│   ├── intrinsics_calibrator.cpp     • ChArUco 기반 K/dist 캘리브레이션 + 품질 게이트
│   ├── ldc_checker.cpp               • 렌즈 왜곡 보정 실시간 정합 진단 (LDC_CHECK)
│   ├── homography_mapper.cpp         • 앵커 8개로 온카메라 H 계산·검증·영속화
│   ├── marker_plane.cpp              • H_marker — 로봇 마커가 실제로 놓인 평면의 H
│   ├── calib_view_store.cpp          • 채택된 캘리브레이션 뷰의 JPEG 인메모리 보관
│   ├── pose_sender.cpp               • 대시보드 TCP 클라이언트 (CAM_POSE, IF-TCP-003)
│   ├── snapshot_sender.cpp           • LDC 스냅샷 전용 연결·포트로 1회성 신뢰 업로드
│   ├── central_tls_sender.cpp        • 중앙 서버 논블로킹 TLS 클라이언트 (role=CCTV)
│   └── toojpeg.cpp                   • 의존성 없는 경량 JPEG 인코더 (제3자 코드)
├── inc/app_config.h                  ➔ 모든 기능 플래그·기준값의 단일 기준점
├── Makefile                          ➔ SoC별 크로스 컴파일 (기본 타깃 cv2x)
├── opencv_cross/ & openssl_cross/    • OpenCV·OpenSSL 크로스 빌드 스크립트
└── config.sh.example                 • 대시보드 실행 포트 템플릿
```

---

##  3. 인터페이스 규격

### 1) 관리자 대시보드 링크 — TCP `POSE_SERVER_IP:7000`

개행으로 구분된 JSON 한 줄. 검출된 마커마다 한 줄, 미검출 프레임에는
`confidence:0` 하트비트를 보냅니다.

```json
{"type":"CAM_POSE","seq":1234,"t":…,"t_frame":…,"w":1920,"h":1080,
 "t_det":12.3,"id":49,"confidence":1.0,
 "corners":[{"x":…,"y":…}, …],
 "world":{"x":1500.0,"y":1000.0,"theta":37.5}}
```

| 필드 | 의미 |
| :--- | :--- |
| `seq` / `t` / `t_frame` | 프레임 시퀀스 / 송신 시각 / 프레임 시각 (epoch ms) |
| `w` / `h` / `t_det` | 프레임 해상도 / `detectMarkers()` 소요 (ms) |
| `confidence` | `1.0` = 검출, `0` = 하트비트 (`corners` 비어 있음) |
| `world` | **H가 계산돼 있을 때만** 추가 — 중심의 월드 mm 좌표와 헤딩(deg) |

역방향으로는 같은 소켓에서 명령을 받습니다(§3-3).

### 2) 중앙 관제 서버 링크 — TLS `CENTRAL_TLS_SERVER_IP:9000`

| 타입 | 방향 | 내용 |
| :--- | :---: | :--- |
| `HELLO` | ▶ | 역할 선언 (`role=CCTV`) |
| `POS` | ▶ | raw 코너 좌표 — 로봇 마커(`CENTRAL_ID`)만 필터링 |
| `H_MATRIX` | ▶ | 계산된 호모그래피 전달 |
| `CALIB_START` | ◀ | 제한적으로 수신 처리 |

> 이 링크는 **legacy 클라이언트**입니다. 중앙 TLS 프로토콜 v2 전체는 구현 완료
> 규격이 아니라 설계 초안이므로, 현재 코드와 목표 규격을 혼동하지 않습니다.

### 3) 런타임 명령 (54종)

재빌드 없이 대시보드에서 투입합니다.

| 그룹 | 주요 명령 |
| :--- | :--- |
| **내부 파라미터** | `CALIB_K_START/CAPTURE/COMPUTE/SAVE/UNDO/GATE/CONFIG/PROFILE_LIST/UPLOAD` |
| **왜곡 진단** | `LDC_CHECK_START`, `LDC_CHECK_STOP`, `LDC_SNAPSHOT`, `K_LOAD` |
| **호모그래피** | `HG_SET`, `HG_SAVE`, `HG_QUERY`, `HG_COORD_MODE`, `HG_CHARUCO_START`, `HG_SNAPSHOT` |
| **앵커·검증** | `ANCHOR_SET`, `ANCHOR_SET_ALL`, `ANCHOR_SET_SLOT`, `ANCHOR_QUERY`, `VALIDATION_SET`, `VALIDATION_QUERY` |
| **마커 평면** | `MARKER_HEIGHT`, `CAMERA_HEIGHT`, `MARKER_PLANE_SAVE`, `MARKER_PLANE_QUERY` |
| **검출 튜닝** | `ROI_SET`, `DYNROI`, `DYNROI_IDS`, `DETECT_PARAM`, `DETECT_ENABLE`, `TUNE_SAVE/QUERY/CLEAR` |
| **중앙 서버** | `CENTRAL_LINK`, `CENTRAL_ID`, `CENTRAL_POS`, `CENTRAL_HMATRIX`, `CENTRAL_QUERY` |
| **진단** | `ARUCO_SCAN`, `RAW_FPS_TEST`, `GET_RAW_RES`, `IMAGE_QUERY`, `SHELL`(무인증) |

---

##  4. 빌드 및 배포 가이드

### [Step 0] 전제 조건

* **OpenSDK 5.00 크로스 빌드 환경** — `Makefile`이 `/opt/opensdk/opensdk-5.00/`
  아래의 툴체인을 참조합니다.
* **`IPCameraManifest.xml`과 서명 번들** — `opensdk_packager`가 요구하지만 SDK가
  제공하는 산출물이라 저장소에서 제외했습니다. OpenSDK 샘플에서 가져와 작업 사본에
  둡니다.
* **크로스 빌드된 OpenCV·OpenSSL** — `opencv_cross/`, `openssl_cross/`의 스크립트로
  먼저 준비합니다.

### [Step 1] 배포 설정 템플릿 복사

```bash
cp inc/app_config_local.h.example inc/app_config_local.h
cp config.sh.example config.sh
```

`app_config.h`의 `POSE_SERVER_IP`·`CENTRAL_TLS_SERVER_IP`는 저장소에 `127.0.0.1`
자리표시자로 들어 있습니다. **실제 주소는 Git에서 제외된 `app_config_local.h`에**
넣으며, 컴파일 시점에 박히므로 바꾸면 `.cap`을 다시 빌드해야 합니다.

### [Step 2] 빌드 및 설치

```bash
make                 # 기본 타깃 bin/cctv_app_cv2x
opensdk_packager
```

생성된 `cctv_app.cap`을 카메라 웹 UI의 Open Platform 화면에서 설치하고 앱을
재시작합니다. 대시보드 호스트에서 `.cap`을 빌드하지 않습니다.

> 카메라 앱의 온카메라 웹페이지는 SDK 샘플 템플릿에서 손대지 않았고 실제 기능이
> 없어 저장소에 포함하지 않았습니다. 관리자 UI는 `Server/admin_console/`입니다.

### [Step 3] 관리자 대시보드 실행

```bash
cd ../Server/admin_console && python3 -u web_gui.py 7000 8083 7001
```

인자는 순서대로 **pose·명령 포트 / HTTP 포트 / 스냅샷 포트**입니다. `start.sh`를
쓰면 `config.sh`의 값으로 백그라운드 실행됩니다. 대시보드는 캘리브레이션,
호모그래피, 마커 검출, 서버 송신, Shell 탭을 제공합니다.

---

##  5. 안전 및 기준 파일

* **`ENABLE_SHELL_CMD=1`이 기본값입니다** — 인증 없는 실험실 진단 기능이므로
  외부망이나 운용 앱에 그대로 포함하지 않습니다.
* 실제 주소·계정·인증서·키는 Git과 문서에 넣지 않습니다. 배포별 값은
  `inc/app_config_local.h`와 `config.sh`에 두며 둘 다 Git에서 제외됩니다.
* **코드와 문서가 다르면 `inc/app_config.h`와 실제 구현을 우선합니다.**
