# ArucoPoseWiseAI (CCTV_4ch_wiseai)

Hanwha OpenSDK 5.00 카메라(PNM-C16083RVQ, 4센서)에서 ArUco 마커로 로봇 위치를
잡고, WiseAI(카메라 내장 사람 검출 AI 패키지)가 보내는 사람 bbox와 결합해
사람-로봇 근접도를 판정하는 카메라 내부 앱이다. 이 디렉터리는 카메라에서 도는
`.cap` 앱 소스만 담는다.

> 문서 대조일: 2026-08-25
> 기능 플래그·기본값의 최종 기준: `app/src/sample_component/includes/app_config.h`
> 배포별 네트워크 값·자격증명은 Git에서 제외됨 (아래 "제외된 것들" 참고)

## 시스템 구성

```text
PNM-C16083RVQ / ArucoPoseWiseAI.cap
  ├─ 4채널 raw NV12 프레임에서 ArUco 검출 (로봇 마커, 기본 ID 49)
  ├─ WiseAI MetadataManager ── ONVIF 메타데이터(사람 bbox) 구독
  ├─ 픽셀→월드 호모그래피로 로봇/사람 위치를 동일 좌표계에 투영
  ├─ ProximityGuard ── 거리 기반 안전/주의/위험 상태머신 (히스테리시스 + 디바운스)
  ├─ TCP pose ────────────────────> 비전 서버(POSE_SERVER_IP:POSE_SERVER_PORT)
  └─ TLS HELLO/POS/ZONE_EVENT ────> 중앙 서버(CENTRAL_TLS_SERVER_IP:9000)
```

WiseAI 자체는 별도 앱(카메라에 같이 설치되는 Hanwha 패키지)이며, 이 앱은 그
메타데이터를 구독만 한다 — WiseAI를 구현하지 않는다.

## 현재 구현

- `DICT_4X4_50` ArUco 검출 (동적 ROI, per-lens duty cycle 조절)
- ChArUco 기반 K/dist 캘리브레이션과 캡처 품질 게이트
- 픽셀→월드 호모그래피 계산·검증, `raw`/`undistort` 좌표 모드
- 로봇 마커 높이 시차(parallax) 보정
- WiseAI ONVIF 메타데이터 파싱 (사람 bbox, object_id, IVA 구역 Enter/Exit)
- `ProximityGuard`: 사람-로봇 거리 기반 안전(Safe)/주의(Caution)/위험(Danger)
  상태머신. IVA 판정 기준점은 bbox **중심**이며 별도 지연 없음
- 중앙 서버 TLS 링크: HELLO/POS/H_MATRIX/ZONE_EVENT 송신, 명령 파서 분리,
  Floor 캘리브레이션 자동 전송
- `GET /status`: 채널별 검출 상태, 근접 판정, 링크 상태를 JSON으로 노출
  (`ENABLE_STATUS_PAGE`)

## 빌드와 배포

카메라 설치·재빌드는 되돌리기 어려운 작업이라 **먼저 확인 후 진행**한다.

```bash
cp camera.env.example camera.env   # 실제 카메라 비밀번호 입력
./build_install.sh -v <버전>        # -v 없으면 이전 cmake 캐시의 버전이 남을 수 있었으나
                                    # build_install.sh:206 수정 후로는 항상 명시값으로 고정됨
```

`build_install.sh`는 OpenSDK 크로스 빌드 컨테이너를 돌려 `.cap`을 만들고, 카메라
Open Platform API로 직접 설치까지 한다. 이 저장소만으로는 `.cap`을 만들 수
없다 — `opensdk_packager`가 요구하는 서명 번들과 `IPCameraManifest.xml`을
이 저장소에서 제외했기 때문이다(아래 참고). OpenSDK 샘플에서 가져와 작업
사본에 둔다.

## 자리표시자로 바꿔야 하는 값

`app/src/sample_component/includes/app_config.h`의 다음 값은 원본에서 컴파일
시점에 박히는 내부 배포 주소다. 이 저장소엔 원본 그대로 올라가 있으므로, 공개
저장소 기준을 엄격히 적용한다면 별도로 자리표시자로 바꿔야 한다(아직 안
바뀜 — TODO):

- `POSE_SERVER_IP` — 비전 서버(RPi 대시보드) 주소
- `CENTRAL_TLS_SERVER_IP`, `CENTRAL_TLS_SERVER_IP_FALLBACK` — 중앙 서버 주소

## 제외된 것들과 이유

| 대상 | 제외 이유 |
|---|---|
| `camera.env`, `pi.env`, `sky.env`, `.env` | 실제 접속 비밀번호. `*.example`만 커밋 |
| `config/app_manifest.json` | `SSHPassword` 평문 포함 |
| `app/res/cert/` | `app_skel.pem`이 개인키 (OpenSDK 템플릿과 동일 바이트지만 원칙상 제외) |
| `opensdk_install`, `app/bin/`, `app/libs/`, `app/build*/`, `*.cap` | SDK 바이너리/빌드 산출물, 재생성 가능 |
| `builds/` | 로컬 빌드 보관 이력 (`.cap` + `INDEX.md`) |
| `docs/`, `rpi/`, `tools/`, `backups/`, `bench/` | 로컬 운용 문서·도구·특정 카메라 실측값. 저장소가 아니라 그 장비를 다루는 사람 손에 있어야 함 |
| `todo.md`, `precheck_merge.sh` | 로컬 작업 메모/이 VM 절대경로가 박힌 스크립트 |

## 안전과 기준 파일

- `ENABLE_SHELL_CMD=1`이 기본값이다 — 인증 없는 진단 기능이므로 외부망·운용
  환경에 그대로 배포하지 않는다.
- 실제 주소·계정·인증서·키는 Git과 문서에 넣지 않는다.
- 코드와 문서가 다르면 `app_config.h`와 실제 구현을 우선한다.
