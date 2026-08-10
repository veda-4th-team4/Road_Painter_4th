# Qt ↔ 서버: 카메라 주소 계약 (2026-08-07)

**이력**: 서버팀 요청서(오전) → [Qt팀 회신](../../QT_ACTION_ITEMS_REPLY_20260807.md) → 서버 반영(오후)
**Qt**: PR #39 `54530fd` (`fix(client): sync job abort and camera routing with server`)
**서버**: `feature/server` — 전역 `cam_ip` 슬롯, `stream.enabled` 3-상태 계약
**대상 코드**: `Client/src/backend.{h,cpp}`, `Client/src/serverclient.cpp`, `Server/src/{stream_cfg.hpp,router.cpp,user_store.cpp}`

> **결론**: Q-1·Q-1'·Q-2 **양쪽 반영 완료.** 남은 미결은 **Q-3(자격 증명 보관 정책)** 하나다.
>
> ⚠️ **아직 통합 시험은 안 했다.** 양쪽 다 빌드·단위 확인까지만 끝났다. 실제 카메라
> IP 변경, 중계/직결 왕복, 로봇 ABORT 물리 정지는 **미실측**이다 (§4).

---

## 0. 현황

| # | 항목 | 상태 | 남은 일 |
|---|---|---|---|
| **Q-1** | 4채널 직결 URL에 서버 `cam_ip` 적용 | ✅ 양쪽 완료 | 통합 시험 |
| **Q-1'** | 재로그인 때 `/FullHD/` 오접속 차단 | ✅ Qt 완료 | 통합 시험 |
| **Q-2** | 중계 OFF 신호 (`stream.enabled`) | ✅ 양쪽 완료 | 통합 시험 |
| **Q-3** | RTSP 계정·비번 하드코딩 | ⏸ **보류** | **자격 증명 보관 정책 합의** |
| Q-4 | 서브스트림 `/chNs` 정책 | ✅ 현행 유지 확인 | 서브 프로파일 생기면 재협의 |
| 추가 | 작업 취소 `ABORT_DRAW` | ✅ Qt 완료 (서버는 이미 지원) | 통합 시험 |

> 📌 **줄 번호 대신 심볼 이름으로 적는다.** 이 문서는 하루 사이 `backend.cpp`가
> 두 번(PR #38, #39) 바뀌면서 줄 번호가 전부 밀렸다. 함수·멤버 이름은 안 썩는다.

---

## 1. 확정된 계약

### 1-1. `LOGIN_OK.cam_ip` — 계정 → 없으면 전역

카메라는 현장에 **한 대**뿐이라(`user_store.hpp` 맨 위 "한 현장 = 카메라 1대 가정")
주소는 사용자 속성이 아니라 **현장 속성**이다. 캘리브레이션(`calib_latest.json`)과
같은 규약으로 전역 슬롯을 둔다.

```
config/camera.json   ← 전역 카메라 IP (계정과 무관)
    ↓ 계정에 cam_ip 가 없으면 이 값이 내려간다
LOGIN_OK.payload.cam_ip
```

| 상황 | `LOGIN_OK.cam_ip` |
|---|---|
| 계정에 `cam_ip` 있음 | **계정 값** (전역이 덮지 않는다 — 기존 현장 보호) |
| 계정에 없음 / 새 계정 | **전역 값** (`config/camera.json`) |
| 둘 다 없음 | `null` |

**Qt 쪽**: `m_channelUrlTemplate`이 `{ip}` 템플릿이 되어, `channelUrl()`이
`{ip}`←`m_camIp`, `{ch0}`←채널−1, `{ch}`←채널로 치환한다. 그래서 서버의
`camera.json`만 바꾸고 재로그인하면 **재빌드 없이** 새 주소로 붙는다.

```text
rtsp://admin:***@{ip}:554/{ch0}/H.264/media.smp
```

### 1-2. `SET_CAM_IP` — 로그인 없이도 저장된다

`H_MATRIX`와 같은 규약으로, **로그인 여부와 무관하게 전역 슬롯에 먼저** 쓰고
로그인 중이면 계정에도 같이 쓴다. 예전에는 로그인이 없으면 거절해서 설치 기사가
계정을 만들기 전엔 카메라를 등록할 수 없었다.

⚠️ **한 사용자가 바꾸면 전역이 바뀌어 다른 사용자에게도 반영된다.** 카메라가 한
대라 의도한 동작이다. 계정에 `cam_ip`를 고정해둔 사용자만 자기 값을 계속 본다.

### 1-3. `LOGIN_OK.stream` — 3-상태 (Q-2, 제안 A)

| 서버 설정 (`config/stream.json`) | `LOGIN_OK.stream` | Qt 동작 |
|---|---|---|
| 파일 없음 / 파싱 실패 | **필드 없음** | "서버가 모른다" — QSettings 유지 |
| `enabled: false` | `{"enabled":false}` | 저장된 `relayBase` **해제** → 직결 |
| `enabled: true` + `base` | `{"enabled":true,"base":…,"channels":…}` | 그 중계 주소 사용 |

- 🔴 **끌 때 필드를 생략하면 안 된다.** 생략은 "구버전 서버 / 설정 미확정"과
  구분되지 않아 Qt가 저장된 주소를 지울 근거가 없다. 그러면 중계를 내려도 Qt가
  죽은 `:8554`로 계속 접속하고, **PC마다 사람이 [비우기]를 눌러야 했다.**
- `enabled: false`일 때 **`base`는 같이 안 보낸다** — "끄라"는 지시에 주소가
  붙으면 어느 쪽을 따를지 애매해진다.
- `enabled` 키가 없으면 Qt가 **`true`로 읽는다** — 이 플래그 이전 서버/파일과 호환.
- **파싱 실패는 "끔"이 아니라 "모름"** — 깨진 파일 때문에 Qt의 멀쩡한 중계 주소를
  지우면 안 된다.

**구현**: 서버 `StreamCfg::shouldSend()`/`toJson()`(`stream_cfg.hpp`),
Qt `ServerClient`의 `streamInfoReceived(bool enabled, …)` → `Backend`에서
`enabled==false`면 `setRelayBase(QString())`.

**서버 로그로 어느 상태인지 바로 보인다:**

```
중계 rtsp://192.168.0.2:8554        ← enabled=true
중계 끔 - QT에 해제 지시(직결)        ← enabled=false
중계 설정 없음 - QT 설정값 유지        ← 파일 없음/깨짐
```

⚠️ `enabled`는 **서버가 주소를 알려줄지**만 정한다. 중계 프로세스(MediaMTX)는
`relay/start.sh -d`로 따로 띄워야 한다 — **중계를 쓰려면 둘 다** 필요하다.

### 1-4. IP 미설정 안전 처리 (Q-1')

Qt가 넣은 가드다. 서버가 `cam_ip`를 못 주는 상황에서 카메라 계정이 잠기는 것을 막는다.

- `channelMode()`가 **`{ip}` 템플릿이면 `cam_ip`까지 있어야** true — 없으면 4채널
  자체가 안 켜진다. `rtsp://…@:554/…` 같은 깨진 URL을 만들지 않는다.
- `setCamIp()`는 채널 템플릿이 활성이면 **단일채널 `m_rtspTemplate`을 안 쓴다.**
  `m_camIp`만 갱신하고 채널 URL을 다시 조립한다 → 채널 작업 중 로그아웃·재로그인해도
  PNM에 없는 `/FullHD/media.smp`를 열지 않는다.
- 옛 QSettings 마이그레이션: **정확히** 옛 기본값
  (`…@192.168.0.13:554/{ch0}/H.264/media.smp`)만 `{ip}` 템플릿으로 자동 이관한다.
  사용자가 직접 만든 템플릿은 안 건드린다.

> 배경: 한화 카메라는 없는 경로를 반복해서 두드리면 계정을 잠근다
> (`RTSP/1.0 490 Account Blocked`). 잠기면 카메라 웹 UI에서 직접 풀어야 한다.

---

## 2. ⏸ Q-3 — 남은 유일한 미결

RTSP 계정·비밀번호가 소스 기본값에 박혀 있다 (`backend.h`의 `m_rtspTemplate`,
`m_rtspUrl`, `m_channelUrlTemplate` + `backend.cpp`의 QSettings 마이그레이션 문자열).
저장소가 공개 GitHub이므로 **기존 비밀번호는 이미 비밀이 아니다 — 운영 자격 증명은
교체하는 것이 맞다.**

Qt팀 회신의 지적이 타당하다: **서버가 `LOGIN_OK`로 비번을 내려보내는 것만으로는
해결되지 않는다.** TLS 전송은 보호되지만 Qt가 QSettings에 저장하면 로컬 디스크에
평문으로 남는다.

**먼저 정해야 할 것 (셋 중 하나):**

1. OS 자격 증명 저장소 사용 (Windows Credential Manager 등)
2. 배포 PC별 비추적 로컬 설정 파일
3. 실행할 때만 입력하고 영속 저장하지 않음

정책이 정해지면 서버는 바로 맞출 수 있다 — `config/camera.json`을 이미 오브젝트로
만들어 뒀으니 키만 늘리면 된다:

```json
{ "cam_ip": "192.168.0.13", "cam_user": "admin", "cam_pass": "…" }
```

**정책 합의 전까지 서버는 payload에 비밀번호 필드를 추가하지 않는다.**

---

## 3. Q-4 — 서브스트림 (현행 유지)

직결 모드에서는 `sub` 여부와 무관하게 같은 채널 URL을 쓴다. 이 카메라에 실제
저해상도 서브 프로파일이 없다는 전제와 일치하므로 그대로 둔다.

그 결과 2x2 미리보기도 채널별 1920x1080을 디코딩한다. **기능 오류는 아니지만
CPU를 많이 쓰는 구조다.** 카메라에 실제 저해상도 프로파일을 만들고 그 정확한
RTSP 경로를 확인하면 `subUrl()`만 분리하는 후속 최적화가 가능하다.

> 참고: 중계 모드에서는 `ch1s`~`ch4s`가 `ch1`~`ch4`로 **redirect**된다
> (`relay/mediamtx.yml`). 예전에는 이 경로들이 `source: publisher`로 남아 있어
> MediaMTX가 **기동 자체를 거부**했고(`'sourceOnDemand' is useless when source is
> 'publisher'`), 그 바람에 `ch1`~`ch4`까지 통째로 안 떴다. 2026-08-07 수정됨.

---

## 4. 🔴 통합 시험 (아직 아무것도 안 했다)

양쪽 다 **빌드와 단위 확인까지만** 끝났다. 아래는 전부 미실측이다.

### 4-1. 카메라 직결

| # | 시험 | 통과 기준 |
|---|---|---|
| 1 | `config/camera.json`에 PNM IP 설정 후 로그인 | 서버 로그에 `카메라 IP 192.168.0.13` |
| 2 | 4채널 URL 확인 | `{ip}`가 아닌 **실제 IP**로 조립됨 |
| 3 | 서버의 IP만 바꾸고 재로그인 | **재빌드 없이** 새 IP로 연결 |
| 4 | 채널 작업 중 로그아웃 → 재로그인 | `/FullHD/` 접속 시도가 **없어야** 함 |
| 5 | `cam_ip=null` 계정으로 로그인 | 깨진 RTSP URL을 열지 않음, 안내 표시 |
| 6 | 계정에 `cam_ip`가 박힌 기존 계정(`test`) | **계정 값이 전역을 이긴다** |

### 4-2. 중계 ↔ 직결 왕복

| # | 시험 | 통과 기준 |
|---|---|---|
| 1 | QSettings에 옛 `relayBase`가 남은 PC로 로그인 (`enabled:false`) | Qt가 `relayBase`를 **지우고** 직결 채널 URL을 엶 |
| 2 | `enabled:true` + `base`로 바꾸고 로그인 | 중계로 복귀 |
| 3 | `stream.json` 삭제 후 로그인 | 기존 사용자 설정 **유지** (지우지 않음) |
| 4 | 중계 복귀 시 `relay/start.sh -d`까지 실행 | `ch1`~`ch4` 재생됨 |

### 4-3. 작업 취소 (`ABORT_DRAW`)

| # | 시험 | 통과 기준 |
|---|---|---|
| 1 | 접근·직선·ARC 각 실행 중 `작업 취소` | 서버 로그 `CMD ABORT_DRAW -> ROBOT` |
| 2 | 로봇 상태 | 속도 0, 노즐 up, path/pending/latch 초기화 |
| 3 | Qt 수신 | `DRAW_ABORTED.was_active=true` |
| 4 | **전원 재부팅 없이** 같은 도면 재시작 | 성공 |
| 5 | 취소 후 경로 수정·재전송 | 성공 |
| 6 | 서버 또는 로봇 오프라인 상태에서 취소 | Qt 타임아웃(3초) 후 실행 상태 유지, 재시도 가능 |

⚠️ **서버의 `DRAW_ABORTED`는 로봇에 명령을 중계한 직후 오는 ACK이지, 로봇의
물리적 정지 완료 ACK가 아니다.** 실제 모터 정지는 4-3의 2번으로 눈으로 확인해야 한다.

---

## 5. 되돌리기

서버는 전부 **파일 한 줄**로 되돌아간다 (재배포 불필요):

```bash
# 전역 카메라 IP 무시하고 예전처럼 계정 값만 쓰기
rm Server/config/camera.json        # 계정의 cam_ip 는 그대로 살아있다
```

```bash
# 중계로 복귀 — 두 가지 모두 필요
# 1) config/stream.json 의 "enabled" 를 true 로
# 2) 중계 프로세스 기동
cd Server/relay && ./start.sh -d
```

Qt는 설정 화면의 **중계 주소 칸**이 그대로 스위치다 — 채우면 중계, 비우면 직결.
단일 채널로 완전히 돌아가려면 `m_channelUrlTemplate`까지 비워야 한다
(`channelMode()` 주석 참고).
