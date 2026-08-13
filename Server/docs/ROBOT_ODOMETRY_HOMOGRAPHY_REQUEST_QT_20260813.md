# Qt팀 요청 — 오도메트리 주행 캘리브레이션 Qt 개시 지원

- 작성일: 2026-08-13
- 수신: Qt팀 (Client)
- 발신: 서버팀
- 전체 설계: [`ROBOT_ODOMETRY_HOMOGRAPHY.md`](ROBOT_ODOMETRY_HOMOGRAPHY.md)
- wire 규격: [`ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md`](ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md)
- 기존 계약: `QT_HOMOGRAPHY_SERVER_CONTRACT_2026-08-10.md`, 회신 [`QT_HOMOGRAPHY_REPLY_20260810_SERVER.md`](QT_HOMOGRAPHY_REPLY_20260810_SERVER.md)

---

## 0. 요약

오도메트리 주행 캘리브레이션(`method:"robot_motion"`)은 **v1에서 관리자 창
전용**으로 확정했고, 서버가 Qt 요청을 `unsupported_from_qt`로 명시 거절하고
있습니다 ([`router_calib.cpp:175`](../src/router_calib.cpp)). 이것을 Qt에서도
시작할 수 있게 엽니다.

**CCTV·로봇은 변경 없습니다.** 양쪽 다 개시자가 누구인지 모르고, 받는 메시지가
완전히 동일합니다. 서버와 Qt만의 변경입니다.

| # | Qt 작업 | 성격 |
|---|---|---|
| 1 | `CALIB_START`에 `method`/`m_cm`/`n_cm`/`start_corner` 4필드 추가 | **수정** |
| 2 | 캘리 시작 UI에 가로·세로·회전방향 입력 + 로봇 배치 안내 | **추가** |
| 3 | 대기 화면이 `CALIB_PROGRESS`의 신규 `point_index` 필드를 표시 | 추가 |
| 4 | `CALIB_FAIL`의 신규 `reason` 6종 문구 | 추가 |
| 5 | 대기 한도를 기존 5분에서 재검토 (§4) | **수정** |
| 6 | `busy` 거절 시 `owner` 필드로 "관리자 창이 쓰는 중" 구분 (§3) | 추가 |

**Qt가 안 해도 되는 것**: 경로 계산, 정지점 위치, 캡처 타이밍, H 산출 — 전부
서버/카메라 몫입니다. 기존 정적 앵커 방식과 동일하게 Qt는 **"시작/중단"과
"사각형 크기·방향"만** 보냅니다.

---

## 1. `CALIB_START` — 4필드 추가 (수정)

### 지금 Qt가 보내는 것

```json
{"type":"CMD","payload":{"cmd":"CALIB_START","ch":2,"request_id":"qt-1723..."}}
```

### 오도메트리 방식으로 보낼 것

```json
{"type":"CMD","payload":{
  "cmd":"CALIB_START", "ch":2, "request_id":"qt-1723...",
  "method":"robot_motion", "m_cm":90, "n_cm":60, "start_corner":"bottom_left"}}
```

| 필드 | 타입 | 필수 | 비고 |
|---|---|---|---|
| `ch` | int | ✅ | 1-based. **Qt 개시에는 필수** (생략 시 `invalid_channel`) |
| `request_id` | string | ✅ | 멱등 처리와 세션 매칭의 정본. 생략하면 재전송 시 로봇이 같은 주행을 두 번 합니다 |
| `method` | string | — | 지금 보내는 값 그대로 두세요. **판별자가 아닙니다** (아래 §1-0) |
| `m_cm` | number | ✅ | 가로. 기본 90 |
| `n_cm` | number | ✅ | 세로. 기본 60 |
| `start_corner` | string | ✅ | `"bottom_left"`(반시계 CCW) \| `"top_left"`(시계 CW) |

### 1-0. 🔴 확인해주세요 — 기존 캘리 버튼이 2026-08-12부터 막혀 있었습니다

서버 코드를 고치면서 발견했습니다. **`method:"robot_motion"`이 두 규격에서 서로
다른 뜻으로 쓰이고 있었습니다:**

| 규격 | 뜻 |
|---|---|
| 2026-08-10 Qt 계약 (`protocol.hpp:259`) | Qt가 **정적 앵커** 요청에 싣는 값 — 지금 Qt가 보내는 그것 |
| 2026-08-12 오도메트리 wire 스펙 | **오도메트리 주행** 방식의 판별자 |

그래서 2026-08-12 이후 **Qt가 보내는 정상적인 정적 앵커 `CALIB_START`가 전부
오도메트리 요청으로 해석돼 `CALIB_FAIL{reason:"unsupported_from_qt"}`로 거절되고
있었습니다.** 서버의 회귀 테스트(`calib_session_test` T1a)로 재현했습니다.

**서버에서 고쳤습니다.** 판별자를 오도메트리 전용 필드(`m_cm`/`n_cm`/`start_corner`)의
존재 여부로 바꿨습니다. Qt는 **아무것도 고칠 필요가 없습니다** — 지금 보내는
정적 앵커 요청은 그대로 정적 앵커로 처리되고, 위 세 필드를 실으면 오도메트리로
갑니다.

다만 확인 부탁드립니다: **그동안 Qt 화면에서 캘리브레이션이 실패하고 있었나요?**
- 실패하고 있었다면 → 이 수정으로 복구됩니다
- 정상 동작했다면 → Qt가 `method`를 안 싣고 있다는 뜻이라, 저희가 파악한 계약과
  실제 구현이 다릅니다. 실제로 보내는 payload를 알려주세요

### 🔴 필드 이름을 바꾸지 마세요

회전 방향이 `start_corner`라는 이름에 실려 있는 게 어색하지만, 서버는 이
`CALIB_START` **원본 메시지를 그대로 ROBOT과 CCTV에 중계**합니다
([`router_odocalib.cpp:66`](../src/router_odocalib.cpp)). `direction:"cw"` 같이
바꿔 보내면 양쪽이 못 읽습니다. Qt UI에서 "좌회전/우회전"으로 보여주고 전송
시점에만 매핑해 주세요.

| UI 표시 | 전송값 | 서버 내부 |
|---|---|---|
| 반시계 / 좌회전 | `"bottom_left"` | `ccw = true` |
| 시계 / 우회전 | `"top_left"` | `ccw = false` |

### 값 제약 (서버가 거절하는 조건)

- `m_cm`, `n_cm` > 0
- **각 변의 절반**이 `min_move_m`(기본 0.01m = 1cm) 이상 → 즉 `m_cm`, `n_cm`가
  각각 **2cm 이상**. 사각형의 각 변을 반으로 쪼개 op을 만들기 때문입니다
  (wire 스펙 §2-1)
- 위반 시 `CALIB_FAIL{reason:"invalid_param"}`

현장 기본값 90×60을 UI 기본값으로 두시고, 상한은 카메라 화각에 달려 있으므로
서버가 강제하지 않습니다 — 사각형이 화면 밖으로 나가면 캡처가
`marker_not_found`로 줄줄이 실패합니다.

---

## 2. Qt가 받게 될 메시지

### 2-1. 세션 흐름

```
Qt  → Server : CMD CALIB_START{method:"robot_motion", ...}
Server → Qt  : CHANNEL_OK{ch, calib}          (활성 채널 전환 통지, 기존과 동일)
Server → Qt  : CALIB_STARTED{ch, request_id, msg}
                 ↓ 로봇이 사각형을 돕니다 (2~4분)
Server → Qt  : CALIB_PROGRESS{ch, request_id, point_index, total, ...}  × 9
                 ↓ 카메라가 H 계산
Server → Qt  : H_MATRIX{ch, request_id, calib:{...}}   ← 성공 종결
        또는   CALIB_FAIL{ch, request_id, reason, msg} ← 실패 종결
        또는   CALIB_CANCELLED{ch, request_id, msg}    ← 취소 종결
```

**종결 응답 3종 중 반드시 하나가 옵니다.** 기존 정적 앵커 방식과 같은 불변식이고,
서버 쪽 이탈 경로(검증 실패·피어 이탈·타임아웃·취소 ACK 불발)가 전부 전송으로
끝나도록 되어 있습니다 ([`router_calib.cpp:13`](../src/router_calib.cpp) 주석).

기존 대기 화면 로직을 그대로 재사용하시면 됩니다. **`method`에 따라 Qt가 다르게
처리할 것은 대기 시간(§4)과 진행률 표시(§2-2)뿐입니다.**

### 2-2. `CALIB_PROGRESS` — 신규 필드 (추가)

정적 앵커 방식에서는 카메라가 진행률을 올려주지만, 오도메트리 방식은 카메라가
`CALIB_PROGRESS`를 보내지 않습니다. **서버가 정지점 진행에서 합성해 보냅니다.**

```json
{"type":"CALIB_PROGRESS","payload":{
  "ch":2, "request_id":"qt-...", "phase":"driving",
  "point_index":3, "total":9, "valid":3}}
```

| 필드 | 의미 |
|---|---|
| `phase` | `"driving"`(주행·캡처 중) \| `"solving"`(주행 끝, 카메라 계산 중) |
| `point_index` | 0~8. 방금 캡처를 마친 정지점 |
| `total` | 9 고정 |
| `valid` | 지금까지 유효한 대응점 수. **6개 미만으로 끝나면 세션 실패**입니다 |

주행이 2~4분 걸리므로 **진행률 없이 빈 대기 화면을 두면 조작자가 멈춘 걸로
오해합니다.** "9개 지점 중 n번째" 정도면 충분합니다. `valid`가 `point_index`보다
많이 뒤처지면 카메라가 마커를 놓치고 있다는 뜻이라, 경고로 띄워 주시면 조작자가
주행이 다 끝나기 전에 중단할 수 있습니다.

### 2-3. `CALIB_FAIL` — 신규 `reason` (추가)

기존 reason(`busy`/`timeout`/`robot_offline`/`cctv_offline`/`invalid_channel`/
`internal_error`/`cancel_failed`)은 그대로 오고, 아래가 추가됩니다.

| reason | 뜻 | 조작자 안내 문구(제안) |
|---|---|---|
| `invalid_param` | m/n/start_corner 값이 규격 밖 | "가로·세로 값을 확인하세요 (각 2cm 이상)" |
| `too_few_points` | 유효 대응점 6개 미만 | "카메라가 로봇을 충분히 인식하지 못했습니다. 조명과 사각형 크기를 확인하세요" |
| `fit_failed` | 카메라 H 피팅 실패 | "호모그래피 계산에 실패했습니다. 다시 시도하세요" |
| `no_intrinsics` | 카메라 K/D 미보정 | "카메라 내부 보정(체커보드)을 먼저 완료하세요" |
| `capture_timeout` | 카메라가 캡처 응답 없음 | "카메라 응답이 없어 중단했습니다" |
| `preempted` | 관리자 창이 세션을 회수 (§3-3) | "관리자가 캘리브레이션을 중단했습니다" |

`detect_off` / `session_refused` / `store_failed` / `session_conflict`도 카메라가
올릴 수 있습니다. **모르는 reason은 `msg` 필드를 그대로 띄우는 fallback**을
두세요 — 카메라 쪽 사유가 계속 늘고 있습니다.

### 2-4. `H_MATRIX` — 번들 확인 (추가)

성공 시 오는 번들에 `method:"robot_motion"`이 실립니다. 나머지 필드는 기존
정적 앵커 방식과 동일한 스키마입니다 (wire 스펙 §6). Qt가 top-view에 쓰는
`H_floor`/`unit:"mm"`도 그대로입니다.

**다만 `origin_mm`/`canvas_mm`가 세션마다 바뀝니다** — 사각형의 원점이 그 세션의
로봇 출발 위치이기 때문입니다. Qt가 이 값을 캐시하고 있다면 갱신해야 합니다.

---

## 3. 세션 소유권 — 관리자 창과의 상호배제

캘리 개시자가 관리자 창과 Qt 둘이고, **로봇은 한 대**입니다. 두 세션이 동시에
돌면 서로의 주행을 자기 관측으로 착각합니다.

### 3-1. 지금 이미 보장되는 것

서버의 모든 메시지 처리는 `mtx_` 하나로 직렬화됩니다
([`router.cpp:6`](../src/router.cpp)). 그래서 **"관리자 창과 Qt의 `CALIB_START`가
동시에 도착해 둘 다 통과하는" 경합은 구조적으로 불가능**합니다. 채널 전환과
세션 개시도 같은 락 안에서 한 덩어리로 끝납니다.

즉 **잠금 자체는 이미 있고, 빠진 것은 "누가 쥐고 있는지"(소유권)** 입니다.

### 3-2. 추가되는 소유권 규칙

서버에 세션 소유자를 명시적으로 둡니다 (`calibOwner_ ∈ {NONE, QT, ADMIN}`,
기존 `calibFromQt_`를 대체).

| 상황 | 서버 동작 | Qt에 가는 것 |
|---|---|---|
| 세션 없음 → Qt가 시작 | owner = QT | `CALIB_STARTED` |
| **Qt 세션 중 → Qt가 또 시작** (같은 `request_id`) | 멱등 — 새 주행 안 만듦 | `CALIB_STARTED` 재전송 |
| **Qt 세션 중 → Qt가 또 시작** (다른 `request_id`) | 거절 | `CALIB_FAIL{busy, owner:"QT"}` |
| **관리자 세션 중 → Qt가 시작** | 거절 | `CALIB_FAIL{busy, owner:"ADMIN"}` |
| **Qt 세션 중 → 관리자가 시작** | 거절 | (관리자 창에만 통지) |
| 도색 작업 중 → 누구든 시작 | 거절 | `CALIB_FAIL{busy}` (owner 없음) |

**Qt 작업**: `busy` 거절에 `owner` 필드가 실립니다. `"ADMIN"`이면 "관리자 창에서
캘리브레이션이 진행 중입니다" 로, `"QT"`면 "이미 진행 중입니다" 로 구분해
주세요. 필드가 없으면 도색 작업 중이라는 뜻입니다. (없는 경우를 대비해 기존
`msg` fallback은 유지)

### 3-3. 🔴 취소 권한과 관리자 선점

취소는 **소유자만** 할 수 있는 것이 원칙입니다. 다만 예외를 하나 둡니다.

> **관리자 창은 Qt 세션을 강제로 회수할 수 있습니다** (`CALIB_CANCEL{force:true}`).

로봇이 실제로 바닥을 굴러다니는 중이고, Qt 단말 앞에 사람이 없거나 Qt가 응답
불능일 수 있습니다. 그 상황에서 관리자가 로봇을 못 세우면 안 됩니다.

- 이때 Qt는 `CALIB_CANCELLED`가 아니라 **`CALIB_FAIL{reason:"preempted"}`** 를
  받습니다. 조작자가 누르지도 않은 취소가 성공한 것처럼 보이면 안 되기 때문입니다.
- 반대 방향(Qt가 관리자 세션 취소)은 허용하지 않습니다 — `CALIB_FAIL{not_owner}`.
  Qt 화면에는 관리자 세션이 보이지도 않으므로 정상 조작으로는 발생하지 않습니다.
- **ESTOP은 소유권과 무관하게 항상 통합니다.** 긴급 정지는 권한 검사 대상이
  아닙니다.

**Qt 작업**: `preempted`와 `not_owner` reason 문구 추가 (§2-3 표).

### 3-4. Qt 연결이 끊기면

현재 서버는 ROBOT/CCTV 연결만 보고 캘리 세션을 접습니다 — **QT가 끊겨도 세션이
살아있습니다** ([`router.cpp:49`](../src/router.cpp)가 QT는 조기 return).
정적 앵커 방식에서는 로봇이 움직이지 않아 큰 문제가 아니었지만, 오도메트리
방식에서는 **주인 없는 로봇이 계속 사각형을 그립니다.**

서버에서 "owner == QT인 세션 중 QT 연결 해제 → 안전 정지"를 추가합니다.
**Qt 작업**: 재접속 후 진행 중이던 캘리를 이어받으려 하지 마세요. 세션은 이미
접혀 있습니다. 재접속 시 대기 화면을 풀고 처음부터 다시 시작하는 UX가 맞습니다.

---

## 4. 대기 한도 — 🔴 Qt의 5분과 충돌합니다

### 문제

`calib_odo_timeout_ms`는 **300000(5분)** 이고, 여기에는 상한 클램프가 **일부러
빠져 있습니다** ([`params.hpp:172`](../src/params.hpp)):

> "이 방식은 Qt가 트리거하지 않으므로 'Qt 5분보다 짧아야 한다'는 제약이 없다"

Qt를 붙이면 이 전제가 무너집니다. 게다가 오도메트리 세션은 **주행이 끝난
뒤에도(`CALIB_DONE`) 카메라가 H를 계산하는 시간이 더 필요**합니다.

```
[Qt 대기 화면 열림] ──── 주행 2~4분 ──── CALIB_DONE ──── 카메라 계산 ──── H_MATRIX
                        └────────── 이 전체가 Qt 한도 안에 들어와야 함 ──────────┘
```

정적 앵커 방식(`calib_timeout_ms` 기본 180초)보다 훨씬 오래 걸립니다.

### 서버 조치

- Qt 개시 세션에 한해 전체 예산에 상한을 겁니다 (주행 데드라인 + 결과 대기 <
  Qt 한도).
- 주행 종료(`CALIB_DONE`) 후 결과를 기다리는 구간을 별도 파라미터로 둡니다
  (`calib_odo_result_wait_ms`, 기본 60초 제안).

### Qt 작업 (확인 필요)

**Qt 자체 대기 한도 5분을 유지할 수 있는지 알려주세요.** 90×60cm 사각형 기준
로봇 주행이 0.05m/s로 둘레 3m + 회전 3회 + 정지점 9곳의 정착 시간이라 **실측
3분 안팎**입니다. 카메라 계산까지 더하면 5분이 빠듯합니다.

| 선택지 | 내용 |
|---|---|
| **A (권장)** | Qt 한도를 `method:"robot_motion"`일 때만 **8~10분**으로 늘림. 서버 데드라인은 그보다 짧게 맞춤 |
| B | 5분 유지. 대신 사각형 상한을 UI에서 제한 (예: 각 변 100cm) |

A로 가면 대기 화면에 **취소 버튼이 반드시 살아 있어야** 합니다 — 10분을 강제로
갇혀 있게 할 수는 없습니다. `CALIB_CANCEL{ch, request_id}`는 지금도 동작하고,
로봇이 실제로 섰다는 확인(`CALIB_STOPPED`)을 받은 뒤에야 `CALIB_CANCELLED`가
갑니다. 즉 **취소 버튼을 눌러도 즉시 화면이 안 풀립니다(최대 5초)** — 그 사이
"정지 중..." 표시가 필요합니다.

---

## 5. UI 요구사항 (추가)

### 5-1. 시작 전 입력

| 항목 | 위젯 | 기본값 |
|---|---|---|
| 가로 | 숫자 입력 (cm) | 90 |
| 세로 | 숫자 입력 (cm) | 60 |
| 회전 방향 | 라디오 2개 | 반시계(좌) |
| 채널 | 기존 채널 선택 재사용 | 현재 채널 |

### 5-2. 🔴 로봇 배치 안내가 반드시 필요합니다

사각형의 **원점과 방향이 로봇의 현재 위치·자세**입니다. 서버가 로봇을 시작
지점으로 이동시키지 않습니다 — 있는 자리에서 바로 사각형을 그리기 시작합니다.

시작 버튼 옆에 다음 안내를 넣어 주세요:

> 로봇을 사각형의 시작 모서리에 놓고, 진행 방향을 맞춘 뒤 시작하세요.
> 사각형 전체가 카메라 화면 안에 들어와야 합니다.

이게 없으면 조작자가 버튼부터 누르고, 로봇이 벽 쪽으로 3m를 그립니다.

### 5-3. 안전

주행 캘리는 **로봇이 실제로 움직이는 작업**입니다. 정적 앵커 방식의 대기
화면과 시각적으로 구분해 주세요 (색·아이콘 등). 조작자가 "가만히 있는 캘리"로
착각한 채 로봇 근처로 걸어가는 상황을 막는 것이 목적입니다.

---

## 6. 지금 서버에 있는 결함 — Qt가 방어해야 할 것

Qt 개시를 열기 전에 서버에서 고칠 항목입니다. 참고로만 남깁니다(Qt 담당 아님).
다만 **③은 지금도 재현되므로** 방어 코드를 권합니다.

| # | 결함 | 위치 |
|---|---|---|
| ① | 오도메트리 세션이 `CALIB_DONE`에서 닫혀, 뒤에 오는 `H_MATRIX`에 `request_id`가 안 실림 | [`router_odocalib.cpp:214`](../src/router_odocalib.cpp) |
| ② | 같은 이유로 카메라의 `CALIB_FAIL{too_few_points, fit_failed}`이 버려짐 → Qt가 타임아웃까지 갇힘 | [`router_calib.cpp:317`](../src/router_calib.cpp) |
| ③ | **관리자 창이 `CALIB_CANCEL`을 누르면, 진행 중인 세션이 없어도 QT에 `CALIB_CANCELLED`가 감** | [`router_calib.cpp:239`](../src/router_calib.cpp) |
| ④ | 실패로 인한 중단(`capture_timeout` 등)이 Qt에는 `CALIB_CANCELLED`로 보임 — "조작자가 취소함"으로 오해 | [`router_calib.cpp:297`](../src/router_calib.cpp) |
| ⑤ | 시작 검증 실패 3종이 로그로만 끝나고 회신이 없음 | [`router_odocalib.cpp:33`](../src/router_odocalib.cpp) |

**③에 대한 방어**: 대기 중이 아닐 때 오는 `CALIB_CANCELLED`, 그리고 자기가 보낸
`request_id`와 다른 종결 응답은 **무시**해 주세요. 정상 동작에서도 늦게 도착한
이전 세션의 응답이 있을 수 있습니다.

---

## 7. 서버 측 변경 목록 (참고)

| # | 내용 |
|---|---|
| 1 | `unsupported_from_qt` 거절 해제, `calibFromQt_` → `calibOwner_` 전환 |
| 2 | 오도메트리 경로에서 `CALIB_STARTED` 전송 |
| 3 | m/n/start_corner 검증을 `startCalib()`으로 올려 `CALIB_FAIL{invalid_param}` 회신 |
| 4 | `CALIB_DONE` 후 세션을 "결과 대기"로 유지 (①②) |
| 5 | 중단 사유를 종결 응답에 반영 — 실패는 `CALIB_FAIL`, 취소만 `CALIB_CANCELLED` (④) |
| 6 | `cancelCalib()`에 origin 인자 + 소유권 검사 + `force` 선점 (③, §3-3) |
| 7 | QT 연결 해제 시 owner=QT 세션 안전 정지 (§3-4) |
| 8 | Qt 개시 시 타임아웃 예산 클램프, `calib_odo_result_wait_ms` 신설 (§4) |
| 9 | `CALIB_PROGRESS` 합성 전송 (§2-2) |
| 10 | wire 스펙 §1·§8·§11 갱신 ("Qt는 트리거하지 않는다"가 거짓이 됨) |

---

## 8. 확인 부탁드립니다

1. **§4 대기 한도** — A(8~10분으로 연장) / B(5분 유지 + 크기 제한) 중 어느
   쪽인가요? 서버 데드라인을 여기에 맞춰야 해서 이것부터 정해야 합니다.
2. **§2-2 진행률** — 제안한 필드(`phase`/`point_index`/`total`/`valid`)로
   충분한가요? 대기 화면에 더 필요한 값이 있으면 알려주세요.
3. **§3-3 관리자 선점** — Qt 세션이 관리자에 의해 회수될 수 있다는 규칙에
   동의하시는지. 대안이 있다면 논의하겠습니다.
4. **§5-2 로봇 배치** — 안내만으로 충분한지, 아니면 서버가 "현재 로봇 위치에서
   사각형이 화면 안에 들어오는지" 사전 검사를 해야 하는지. (사전 검사는 카메라
   협의가 더 필요해 v1 범위 밖으로 두려 합니다.)
