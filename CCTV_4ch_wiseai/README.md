# Road Painter — CCTV 엣지 측위 & WiseAI 근접 감시 앱

본 디렉터리는 **Road Painter**의 인프라 비전 측위 모듈, 즉 한화비전 4채널 CCTV
내부에서 직접 구동되는 **온카메라 엣지 앱(`.cap`)** 소스와 배포 스크립트를
포함합니다. ArUco 마커로 로봇의 위치·헤딩을 산출하고, 카메라 내장 AI 패키지인
**WiseAI**가 송신하는 사람 bbox와 동일 월드 좌표계에서 결합하여 사람-로봇
근접 위험도를 판정합니다.

---

##  1. 핵심 사양 & 성능 지표

### 1) 시스템 사양

| 항목 (Specification) | 수치 / 규격 | 세부 비고 |
| :--- | :--- | :--- |
| **대상 카메라** | **PNM-C16083RVQ** (4센서 멀티디렉셔널) | 칩셋 Ambarella **CV5**, 빌드 파라미터 `SOC=cv5` |
| **빌드 플랫폼** | **Hanwha OpenSDK v26.05.19** | 크로스 빌드 이미지 `opensdk:26.05.19_full` |
| **동시 처리 채널** | **4 채널** (`kMaxChannels`) | 채널별 독립 ROI·호모그래피·근접 상태머신 |
| **영상 입력** | **Raw NV12 Y-plane 직접 접근** | RTSP 디코딩 경유 없이 8비트 그레이 평면을 그대로 사용 |
| **마커 규격** | **`DICT_4X4_50`**, 로봇 마커 ID **`49`** | 앵커·검증 마커 ID와 월드 좌표(mm)는 런타임 지정 (`ANCHOR_SET_ALL`) |
| **캘리브레이션 보드** | **ChArUco 7×5**, A3 가로 (420×297 mm) | 사각 $50\text{ mm}$ / 마커 $35\text{ mm}$, 패턴 350×250 mm |
| **좌표 변환** | **렌즈별 픽셀→월드 호모그래피** | `raw` / `undistort` 두 모드, ChArUco 기반 K·dist 보정 |
| **높이 보정** | **마커 높이 시차(parallax) 보정** | `H_marker` 유도는 `undistort` 모드에서만 성립 |
| **사람 검출 기준점** | **bbox 발끝점** $((left+right)/2,\ bottom)$ | WiseAI 자체 IVA는 bbox **중심** 기준 — 두 판정을 UtcTime으로 대조 |
| **근접 임계값** | **주의 $1500/1800\text{ mm}$, 위험 $700/900\text{ mm}$** | 진입/해제 히스테리시스 + `300 ms` 디바운스 |
| **구역 완충밴드** | **위험 $300\text{ mm}$ / 경고 $500\text{ mm}$** | 작업구역 폴리곤 바깥으로 월드 mm 단위 확장 |
| **비전 서버 링크** | **TCP `7100`** (평문, 영속 연결) | `CAM_POSE` 스트림 + 캘리브레이션 명령 수신 |
| **중앙 서버 링크** | **TLS `9000`** (`role=CCTV`) | 논블로킹, 인증서 IP 검증, 후보 IP 2개 로테이션 |

### 2) 정량 성능 (루트 [README](../README.md) §4 측정치)

| 평가 항목 (Metric) | 초기치 | 최종 결과 | 개선 의미 |
| :--- | :---: | :---: | :--- |
| **카메라 엣지 측위 지연** | $499.8\text{ ms}$ (RTSP) | **$196.4\text{ ms}$ (Raw)** | $304\text{ ms}$ 단축 — 초당 5 fps 실시간 제어 확보 |
| **Dynamic ROI 처리율** | $2.00\text{ fps}$ (CPU 69.5%) | **$10.12\text{ fps}$ (CPU 40.2%)** | 전체 프레임 스캔 → 마커 추종 박스 스캔 |
| **4채널 검출 시간** | $278\text{ ms}$ | **$5.1\text{ ms}$** | 픽셀 수에 비례하는 `detectMarkers()` 비용 절감 |

---

##  2. 시스템 아키텍처 및 디렉터리 구조

### 1) 데이터 흐름

```text
PNM-C16083RVQ / ArucoPoseWiseAI.cap
  ├─ 4채널 Raw NV12 프레임 ─▶ 동적 ROI ─▶ ArUco 검출 (로봇 마커 ID 49)
  ├─ WiseAI MetadataManager ─▶ ONVIF 메타데이터 구독 (사람 bbox, IVA Enter/Exit)
  ├─ 렌즈별 호모그래피 ─▶ 로봇·사람을 동일 월드 좌표계(mm)로 투영
  ├─ ProximityGuard ─▶ 거리 기반 안전/주의/위험 상태머신 (히스테리시스 + 디바운스)
  ├─ TCP  CAM_POSE ──────────────────▶ 비전 서버 (RPi 대시보드, 7100)
  └─ TLS  HELLO/POS/H_MATRIX/ZONE_EVENT ▶ 중앙 관제 서버 (9000)
```

WiseAI는 카메라에 함께 설치되는 별도 한화비전 패키지이며, 본 앱은 그 메타데이터를
**구독만** 합니다 — WiseAI를 구현하지 않습니다.

### 2) 디렉터리 구조

```text
CCTV_4ch_wiseai/
├── app/
│   ├── src/sample_component/           ➔ [앱 본체: OpenSDK 컴포넌트]
│   │   ├── sample_component.cc         • 프레임 콜백·명령 디스패치·/status·전체 배선
│   │   ├── aruco_processor.cc          • NV12 Y-plane ➔ detectMarkers ➔ corners2d
│   │   ├── dyn_roi.cc                  • 마커를 따라가는 적응형 ROI (SEARCH ↔ TRACK)
│   │   ├── intrinsics_calib.cc         • ChArUco K/dist 캘리브레이션 + 캡처 품질 게이트
│   │   ├── homography_mapper.cc        • 픽셀→월드 H 피팅·검증, 렌즈별, 시차 보정
│   │   ├── pose_sender.cc              • 비전 서버 TCP 클라이언트 (영속 연결, IF-TCP-003)
│   │   ├── central_tls_sender.cc       • 중앙 서버 논블로킹 TLS 클라이언트 (role=CCTV)
│   │   ├── central_cmd_parse.cc        • 중앙 CMD 한 줄 ➔ 내부 명령 문자열 (호스트 테스트용 분리)
│   │   ├── wiseai_metadata.cc          • WiseAI ONVIF XML 파싱 (bbox, object_id, IVA 이벤트)
│   │   ├── proximity_guard.cc          • 사람-로봇 근접 상태머신 (입력은 거리 mm 뿐)
│   │   └── includes/app_config.h       • 모든 기능 플래그·기본값의 단일 기준점
│   ├── res/models/                     • OpenSDK 매니페스트 (Raw video ×4, Metadata 구독 ×4)
│   ├── html/index.html                 • 온카메라 상태 페이지 (/status 폴링, app_id 런타임 추출)
│   └── third_party/opencv-aarch64/     • aarch64 정적 OpenCV 6종 (core/imgproc/aruco/calib3d/features2d/flann)
├── build_install.sh                    ➔ 크로스 빌드 ➔ .cap 패키징 ➔ 카메라 설치 (6단계)
├── docker-compose.yml                  • SDK 컨테이너 수동 실행용 (스크립트 없이 빌드할 때)
└── wiseai_core_test.cc                 • proximity_guard + wiseai_metadata 호스트 단위테스트
```

---

##  3. 인터페이스 규격

### 1) 비전 서버 링크 — TCP `POSE_SERVER_IP:7100`

개행으로 구분된 JSON 한 줄. 검출된 마커마다 한 줄, 미검출 프레임에는
`confidence:0` 하트비트를 보내 링크 생존과 프레임 카운터를 유지합니다.

```json
{"type":"CAM_POSE","ch":0,"seq":1234,"t":1756600000000,"t_frame":…,"t_capture":…,
 "queue_ms":12,"w":1920,"h":1080,"t_det":5.1,"id":49,"confidence":1.0,
 "corners":[{"x":…,"y":…}, …]}
```

| 필드 | 의미 |
| :--- | :--- |
| `ch` / `seq` | 렌즈 번호 / 해당 렌즈가 **검출에 성공한** 프레임 카운터 |
| `t` / `t_frame` / `t_capture` | 송신 시각 / 프레임 시각 / 캡처 시각 (epoch ms) |
| `queue_ms` / `t_det` | 콜백 대기 시간 / `detectMarkers()` 소요 (ms) |
| `confidence` | `1.0` = 검출, `0` = 하트비트 (`corners` 비어 있음) |

역방향으로는 같은 소켓에서 캘리브레이션·튜닝 명령을 받습니다(§3-4).

### 2) 중앙 관제 서버 링크 — TLS `CENTRAL_TLS_SERVER_IP:9000`

접속 직후 `HELLO`로 역할을 선언하고, 이후 타입별 JSON을 한 줄씩 올립니다.

| 타입 | 방향 | payload 요지 |
| :--- | :---: | :--- |
| `HELLO` | ▶ | `{"role":"CCTV"}` — TLS 핸드셰이크·인증서 검증 성공 직후 1회 |
| `POS` | ▶ | `ch`, `corners[4][2]` — 로봇 마커(`CENTRAL_ID`)만 필터링해 송신 |
| `H_MATRIX` | ▶ | Floor 캘리브레이션 결과 번들 (완료 시 자동 전송) |
| `ZONE_EVENT` | ▶ | `level`, `alarm_level`, `zone_mm`, `foot_u/v`, `foot_wx/wy`, bbox, `t_ms` |
| `CMD` | ◀ | 서버 명령. `central_cmd_parse()`가 내부 명령 문자열로 변환 |

> **인증서 주의** — 후보 IP는 2개(주소 + 폴백)이며 SSL 검증이 IP에 고정됩니다.
> 인증서 SAN에 없는 주소로는 **fail-closed**로 연결되지 않습니다.

### 3) 상태 조회 — `GET /status`

카메라 Open Platform 경로 `/opensdk/<app_id>/status`로 노출됩니다
(`ENABLE_STATUS_PAGE`). 온카메라 페이지가 이 값을 폴링합니다.

| 키 | 내용 |
| :--- | :--- |
| `app` | `name`, `version`, `built`, `uptime_s`, `channels` |
| `proximity` | 채널별 `state`(safe/caution/danger), `last_transition_ms` |
| `central` / `pose` | 중앙 TLS 링크 상태, 비전 서버 연결 여부 |
| `governor` | duty cycle 조절기 — `duty_pct`, `min`, `max`, `active` |
| `dynroi` / `dets` | 채널별 ROI 상태, 검출 마커·`seen_ids` |
| `homography` / `hg_map` / `anchors` | 렌즈별 H 유효성·피팅 오차, 좌표 변환 단발 조회 |
| `calib` / `board` / `sessions` | K·dist 캘리브레이션 세션, 저장 경로(`persist`) |
| `iva_sync` | WiseAI IVA 구역과 이 앱 좌표계의 정합 결과 |
| `proc` / `video` / `network` | CPU 점유, 프레임 통계, 링크 상태 |

### 4) 런타임 명령 (46종)

재빌드 없이 두 링크 어느 쪽으로도 투입할 수 있습니다.

| 그룹 | 명령 |
| :--- | :--- |
| **내부 파라미터** | `CALIB_K_START/CAPTURE/COMPUTE/SAVE/REVERT/UNDO/GATE/STATUS/QUERY` 외 |
| **호모그래피** | `HG_SET`, `HG_SAVE`, `HG_CLEAR`, `HG_QUERY`, `HG_MAP`, `HG_COORD_MODE` |
| **마커·앵커** | `ANCHOR_SET_ALL`, `ANCHOR_SAVE`, `ANCHOR_QUERY`, `MARKER_HEIGHT`, `CAMERA_HEIGHT` |
| **근접·구역** | `ZONE_MARGIN`, `ZONE_RADIUS`, `ZONE_BANDS`, `ZONE_ALARM_LEVEL`, `IVA_ZONE_SET`, `IVA_SYNC` |
| **중앙 서버** | `CENTRAL_LINK`, `CENTRAL_ID`, `CENTRAL_POS`, `CENTRAL_HMATRIX`, `CENTRAL_QUERY` |
| **진단** | `DET_STREAM`, `SHELL`(무인증 — 운용 배포 시 비활성 권장) |

---

##  4. 전제 조건 및 빌드·설치 가이드

### [Step 0] 전제 조건

* **크로스 빌드 환경** — `opensdk:26.05.19_full` 이미지가 로컬에 있어야 합니다.
  배포본을 받아 `docker load`하는 절차는 한화비전 **Open Platform v26** 문서
  세트의 Programming Guide를 따릅니다(v5 문서 아님).
* **카메라** — Open Platform이 활성화되어 있고, **WiseAI 앱이 설치·채널 설정**된
  상태여야 합니다. WiseAI가 없으면 사람 검출·근접 판정이 동작하지 않습니다.
* **자격증명 파일 2종** — 아래 표 참고. 저장소에서 제외되어 있습니다.

| 채울 파일 | 없으면 | 채우는 법 |
| :--- | :--- | :--- |
| `config/app_manifest.json` | `build_install.sh:28-32`에서 AppName을 못 읽고 즉시 종료 | `AppName`은 `ArucoPoseWiseAI` — 그대로 `.cap` 파일명이 됩니다 |
| `app/res/cert/central_server.crt` | `build_install.sh:171-175`에서 `exit 1` | `CENTRAL_TLS_CERT` 지정 또는 `../Server/certs/server.crt` 준비 |

### [Step 1] 빌드 및 카메라 설치

```bash
cp camera.env.example camera.env     # 카메라 IP / 계정 / 비밀번호 입력
./build_install.sh -v 0.4.5          # 크로스 빌드 ➔ .cap 패키징 ➔ 카메라 설치
```

`build_install.sh`는 SDK 컨테이너에서 `.cap`을 만들고 Open Platform API로 설치까지
수행합니다. 서명·패키징 자산은 SDK 이미지 안에 있으므로 따로 준비할 것이 없습니다.
`-v` 값은 `/status`의 `app.version`으로 노출되며, 생략 시 `0.3.0`으로 고정됩니다
(`build_install.sh:206`).

> **주의** — 카메라 설치·재빌드는 되돌리기 어려운 작업입니다. 실행 전 확인하십시오.

### [Step 2] 배포 주소 확인

`app/src/sample_component/includes/app_config.h`의 아래 값은 **컴파일 시점에
박힙니다.** 다른 환경에 배포하려면 수정 후 다시 빌드해야 합니다.

* `POSE_SERVER_IP` / `POSE_SERVER_PORT` — 비전 서버(RPi 대시보드)
* `CENTRAL_TLS_SERVER_IP`, `CENTRAL_TLS_SERVER_IP_FALLBACK` — 중앙 관제 서버
* `camera.env.example`의 `CAMERA_IP`, `pi.env.example`의 `PI_HOST` — 동일 성격

---

##  5. 테스트 및 검증 도구

### 1) 호스트 단위테스트 (`wiseai_core_test`)

카메라 없이 근접 상태머신과 WiseAI 메타데이터 파서를 검증합니다. CMake 대상이
아니므로 직접 컴파일합니다.

```bash
g++ -std=c++17 -Iapp/src/sample_component/includes wiseai_core_test.cc \
    app/src/sample_component/proximity_guard.cc \
    app/src/sample_component/wiseai_metadata.cc -o /tmp/wtest && /tmp/wtest
# wiseai_core_test: PASS
```

### 2) 실시간 상태 확인 (`/status`)

```bash
curl -s "http://<CAMERA_IP>/opensdk/ArucoPoseWiseAI/status" | python3 -m json.tool
```

* `proximity` : 채널별 근접 판정이 safe/caution/danger 중 무엇인지
* `central` : 중앙 TLS 링크가 online인지, 어느 후보 IP에 붙었는지
* `governor` : duty cycle이 몇 %로 떨어졌는지 (CPU 부하 추적)
* `calib.persist` : 캘리브레이션 파일이 실제로 어느 경로에 쓰이는지

### 3) 좌표 변환 단발 검증 (`HG_MAP`)

호모그래피가 맞게 잡혔는지 픽셀 한 점을 월드 mm로 변환해 확인합니다. 결과는
`/status`의 `hg_map`에 소수점 4자리로 남습니다.

```text
HG_MAP <ch> <px> <py>        ➔ hg_map:{ok, wx_mm, wy_mm, reason}
HG_COORD_MODE <ch> <0|1>     ➔ 0=raw, 1=undistort (시차 보정은 undistort 전용)
IVA_SYNC <ch>                ➔ WiseAI IVA 구역과 이 앱 좌표계의 정합 확인
```
