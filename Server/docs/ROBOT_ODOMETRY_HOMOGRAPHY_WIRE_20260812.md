# 로봇 오도메트리 호모그래피 — Wire 스펙 (확정본)

- 작성일: 2026-08-12
- 상태: **확정. 서버·CCTV 양쪽 구현의 근거 문서.**
- 이 문서는 **wire 형식 상세 규격**이다. 기능 전체(설계·구현 현황·검증 한계)는
  **`docs/ROBOT_ODOMETRY_HOMOGRAPHY.md`(정본)** 을 볼 것.
- 왜 이렇게 정했는지는 아래 문서들에 있다 — 여기서는 되풀이하지 않는다.
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_20260812.md`
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_SERVER_20260812.md`
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_02_20260812.md`

---

## 1. 세션 개시 — 관리자창 / Qt (2026-08-13 갱신)

> **2026-08-12 원문은 "관리자창 전용, Qt는 트리거하지 않는다. v1 범위 밖"이었다.**
> 2026-08-13에 Qt 개시를 열면서 이 절과 §8, §11이 바뀌었다. Qt 쪽 작업 목록과
> 소유권 규칙은 [`ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_QT_20260813.md`](ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_QT_20260813.md)에 있다.
> **CCTV·로봇은 변경 없다** — 개시자가 누구든 받는 메시지가 동일하다.

```
관리자창|Qt → Server:  CMD {cmd:"CALIB_START", ch, request_id,
                            method:"robot_motion", m_cm, n_cm, start_corner}
```

| 필드 | 타입 | 필수 | 비고 |
|---|---|---|---|
| `ch` | int | ✅ | 1-based. **QT는 필수**, ADMIN이 생략하면 현재 활성 채널 |
| `request_id` | string | ✅ | 서버가 세션 내내 이 값을 정본으로 씀 |
| `method` | string | — | `"robot_motion"`. **판별자가 아니다** — 아래 §1-0 참고 |
| `m_cm` | number | ✅ | 가로. 기본 90 |
| `n_cm` | number | ✅ | 세로. 기본 60 |
| `start_corner` | string | ✅ | `"bottom_left"`(CCW) \| `"top_left"`(CW) |

치수 검증은 `startCalib()`이 한다 (`router_calib.cpp`). `m_cm`/`n_cm`가 양수여야
하고, 각 변의 **절반**이 `min_move_m` 이상이어야 한다 — 미만이면 그 op이 경로
생성 필터에 걸려 로봇이 실행할 수 없는 미세 동작이 된다. 위반 시
`CALIB_FAIL{reason:"invalid_param"}` (ADMIN 개시면 로그만).

### 1-0. 🔴 판별자는 `method`가 아니다 (2026-08-13 정정)

**원문(2026-08-12)은 `method:"robot_motion"`을 오도메트리 방식의 판별자로 삼았다.
그것이 버그였다.** 같은 값이 이미 다른 뜻으로 쓰이고 있었기 때문이다:

| 규격 | `method:"robot_motion"`의 뜻 |
|---|---|
| 2026-08-10 Qt 계약 (`protocol.hpp:259`) | Qt가 **정적 앵커** 요청에 싣는 값. "로봇 주행 호모그래피"라는 기능 이름에서 왔다 |
| 2026-08-12 이 문서 (원문) | **오도메트리 주행** 방식의 판별자 |

결과: **2026-08-12부터 Qt의 정상적인 정적 앵커 요청이 전부 오도메트리로 해석돼
`unsupported_from_qt`로 거절됐다.** Qt의 캘리브레이션 버튼이 그날부터 죽어
있었다는 뜻이다. `tools/calib_session_test.cpp`의 T1a가 이 증상을 잡는다.

**판별자를 오도메트리 전용 필드의 존재로 바꿨다:**

```
m_cm | n_cm | start_corner 중 하나라도 있으면  → 오도메트리 주행
셋 다 없으면                                  → 정적 앵커 (method 무시)
```

셋 중 일부만 실려 오면 오도메트리로 보되 치수 검증에서 `invalid_param`으로
거절한다 — 잘못 만든 오도메트리 요청이 조용히 정적 앵커로 흘러가면 안 된다.

이 셋은 정적 앵커 요청에 실릴 이유가 없으므로 **이미 배포된 Qt·CCTV·관리자 창을
하나도 고치지 않아도 된다.** `method`는 그대로 받되 무시한다 (§6의 H_MATRIX
번들에 실리는 `method`는 이것과 별개 필드다 — 그쪽은 "H를 어떻게 측정했나"라서
`"robot_motion"`이 계속 맞다).

서버 → 로봇: `PATH{phase:"calib", ops:[...11개]}` (§3).

**Qt 개시일 때 서버 → Qt**: `CHANNEL_OK` → `CALIB_STARTED` → `CALIB_PROGRESS`(§2-2)
→ 종결 응답. ADMIN 개시일 때는 Qt에 아무것도 보내지 않는다.

### 1-1. 세션 소유권

개시자가 둘이므로 서버가 소유자(`calibOwner_ ∈ {NONE, QT, ADMIN}`)를 기억한다.

- **시작**: 이미 세션이 있으면 `CALIB_FAIL{reason:"busy", owner:"QT"|"ADMIN"}`
- **취소**: 소유자만 가능. 아니면 `CALIB_FAIL{reason:"not_owner"}`
- **강제 회수**: 관리자창의 `CALIB_CANCEL{force:true}`만 예외. 로봇이 굴러가는
  중인데 Qt 단말 앞에 사람이 없을 수 있어서다. 이때 Qt에는 `CALIB_CANCELLED`가
  아니라 `CALIB_FAIL{reason:"preempted"}`가 간다 — 누르지도 않은 취소가 성공한
  것처럼 보이면 안 된다
- **소유자 이탈**: QT 연결이 끊기면 QT 소유 세션은 즉시 안전 정지한다. 예전에는
  `onPeerChange`가 QT를 무시해서 **주인 없는 로봇이 사각형을 계속 그렸다**
- `ESTOP`은 소유권과 무관하게 항상 통한다 (캘리 경로를 타지 않는다)

---

## 2. 메시지 5종

```
Server → CCTV   CALIB_CAPTURE       {ch, request_id, point_index, world_xy_mm:[x,y]}
CCTV   → Server CALIB_CAPTURE_OK    {ch, request_id, point_index, pixel_uv:[u,v], spread_px}
CCTV   → Server CALIB_CAPTURE_FAIL  {ch, request_id, point_index, reason}
Server → CCTV   CALIB_DONE          {ch, request_id, m_mm, n_mm}
CCTV   → Server H_MATRIX            {ch, calib:{...}}        (기존 계약, §6)
```

### 2-2. `CALIB_PROGRESS` — 서버가 합성한다 (2026-08-13, QT 개시 전용)

```
Server → QT     CALIB_PROGRESS      {ch, request_id, phase, point_index, total, valid}
```

**카메라는 이 방식에서 진행률을 올리지 않는다** — 정적 앵커 방식과 다른 점이다.
그런데 주행이 2~4분이라 그동안 Qt 대기 화면이 비어 있으면 조작자는 멈춘 것으로
본다. 서버는 정지점 진행을 정확히 알고 있으므로 그걸로 만들어 보낸다.

| 필드 | 값 |
|---|---|
| `phase` | `"driving"`(주행·캡처 중) \| `"solving"`(주행 끝, 카메라 계산 중) |
| `point_index` | 방금 캡처를 마친 정지점 0~8. 세션 개시 직후는 −1 |
| `total` | 9 고정 |
| `valid` | 지금까지 유효한 대응점 수 |

`valid`가 `point_index`를 따라오지 못하면 카메라가 마커를 놓치고 있다는 뜻이다 —
유효점 6개 미만이면 세션이 실패하므로(§7), Qt가 주행이 다 끝나기 전에 조작자에게
경고할 수 있도록 개수를 같이 싣는다.

| 필드 | 타입 | 단위 | 비고 |
|---|---|---|---|
| `ch` | int | — | 1-based |
| `request_id` | string | — | 세션 개시 때 받은 값 그대로 |
| `point_index` | int | — | 0~8 (§4) |
| `world_xy_mm` | [number,number] | mm | **마커 중심** 좌표 (§3-1) |
| `pixel_uv` | [number,number] | px | **raw** 픽셀 (undistort 전, §5) |
| `spread_px` | number | px | 정지 판정 표준편차. 로그·잔차 대조용 |
| `reason` | string | — | §4-1 표 |
| `m_mm`, `n_mm` | number | mm | `m_cm*10`, `n_cm*10` |

### 2-0. 봉투 — 🔴 CMD 와 독립 타입이 섞여 있다

| 메시지 | 봉투 |
|---|---|
| `CALIB_START`, `SELECT_CHANNEL`, `CALIB_CANCEL` | `{"type":"CMD","payload":{"cmd":"<이름>",...}}` |
| `CALIB_CAPTURE`, `CALIB_DONE`, `CALIB_CAPTURE_OK/FAIL`, `CALIB_STOPPED`, `H_MATRIX` | `{"type":"<이름>","payload":{...}}` |

§1 의 세션 개시(관리자창 → Server)만 CMD 이고, §2 의 5종은 자체 type 이다.
서버 구현이 이미 그렇게 돼 있다(`router_odocalib.cpp`의 `makeMsg("CALIB_CAPTURE", ...)`
vs `makeMsg("CMD", {{"cmd","CALIB_CANCEL"}})`).

> 2026-08-12: 이 규칙이 문서에 없어서 카메라 파서가 `"type":"CMD"` 를 먼저 요구하도록
> 구현됐다. 그 결과 `CALIB_CANCEL` 은 되는데 `CALIB_CAPTURE`/`CALIB_DONE` 만 조용히
> 버려졌고, 첫 통합 주행이 `capture_timeout` 으로 끝났다(서버 로그 18:56:53 → 18:57:08).
> 수신 측은 **봉투를 가리지 말고 명령 이름으로 판별**할 것.

### 2-1. 매칭 키 — `ch` + `request_id` + `point_index` 세 개 전부 일치

하나라도 어긋나면 **무시하고 WARN 로그만** 남긴다 (응답하지 않는다 — 그 메시지는
지금 세션의 것이 아니다). 세션 재시작 시 `point_index`가 0으로 리셋되므로
`request_id`만으로는 이전 세션의 늦은 ack와 새 세션의 대기가 같은 인덱스에서
충돌할 수 있다. 세 값 전부를 키로 쓴다.

---

## 3. 로봇 경로 — `CALIB_START` 중계 + 11-op, 로봇 펌웨어 변경 없음

서버는 세션 개시 시 **`CALIB_START` 원본을 ROBOT/CCTV 양쪽에 중계한 뒤** PATH를
보낸다:

| 수신자 | 하는 일 |
|---|---|
| ROBOT | R-1 핸들러가 `auto_nozzle=0` + `SendControlNozzle(0)` — **캘리 중 노즐 강제 UP** |
| CCTV | 카메라 세션의 시작점 (CCTV팀 2차 회신 §5-B) |

> 🔴 로봇 쪽은 안전 항목이다. PATH 적용은 `manual_nozzle`만 0으로 되돌리고
> `auto_nozzle`은 안 건드리므로, 직전 도색이 노즐을 내린 채 비정상 종료됐다면
> 캘리 주행이 바닥을 칠하며 돈다. 캘리 op에는 nozzle op이 없어서 R-1이 유일한
> 방어선이다.

```
ops = [ MOVE(m/2), MOVE(m/2), TURN(±90°),
        MOVE(n/2), MOVE(n/2), TURN(±90°),
        MOVE(m/2), MOVE(m/2), TURN(±90°),
        MOVE(n/2), MOVE(n/2) ]
```

- `start_corner == "bottom_left"` → `TURN(+90)` (CCW)
- `start_corner == "top_left"` → `TURN(-90)` (CW)
- 모든 op: `hasTarget=false`, `headingDeg=kNoHeading`, `centerAheadByA=false`,
  nozzle op 없음 — **피드백(ALIGN/MORE/DRIFT) 완전 차단**이 op 자체에 이미
  들어있다. `phase=="calib"` 가드(§7)는 이중 방어다.
- 기존 `PATH`/`READY`/`GO`/`PATH_DONE` 핸드셰이크 그대로. 새 PATH 수신 시 로봇은
  **2.5초** 정지 대기 후 `READY(0)` — 출발점의 정착 대기가 별도 구현 없이
  충족된다. 로봇은 새 PATH를 현재 세그먼트가 끝난 뒤 적용한다(IMU yaw 보호).

---

## 4. 9점 좌표표

`m`/`n` 단위 mm (`m_cm*10`, `n_cm*10`). 예시는 `m_cm=90, n_cm=60` →
`m=900mm, n=600mm`.

| `point_index` | `bottom_left` | `top_left` | 캡처 트리거 |
|---:|---|---|---|
| 0 | `(0, 0)` | `(0, 600)` | `READY(0)` |
| 1 | `(450, 0)` | `(450, 600)` | `READY(1)` |
| 2 | `(900, 0)` | `(900, 600)` | `READY(2)` |
| — | (스킵, 즉시 GO) | | `READY(3)` — 직전 op TURN |
| 3 | `(900, 300)` | `(900, 300)` | `READY(4)` |
| 4 | `(900, 600)` | `(900, 0)` | `READY(5)` |
| — | (스킵, 즉시 GO) | | `READY(6)` — 직전 op TURN |
| 5 | `(450, 600)` | `(450, 0)` | `READY(7)` |
| 6 | `(0, 600)` | `(0, 0)` | `READY(8)` |
| — | (스킵, 즉시 GO) | | `READY(9)` — 직전 op TURN |
| 7 | `(0, 300)` | `(0, 300)` | `READY(10)` |
| 8 | `(0, 0)` | `(0, 600)` | `PATH_DONE` (READY 없음) |

**`point_index 8`은 출발점(0)과 라벨이 겹치는 복귀점이다.**
`findHomography` 입력에서 **제외**하고, idx 0 대비 픽셀 차이를 오도메트리
누적 오차(폐합오차)로만 쓴다.

### 4-1. `world_xy_mm`의 정의 — 🔴 가장 중요한 한 줄

> `world_xy_mm` = 그 순간 **마커 중심**의 월드 좌표. 로봇 회전 중심이 아니다.

`marker_offset_m`(기본 0.0, `params.json`)이 0이 아니면 서버가 헤딩 방향으로
보정해 위 표의 좌표를 이동시켜 보낸다:

```
world = center + marker_offset_mm * (cos(heading), sin(heading))
```

카메라는 이 보정을 몰라도 된다 — 받은 좌표를 그 순간 캡처한 픽셀과 그대로
짝지으면 된다.

---

## 5. 픽셀 공간 — raw

`pixel_uv`는 **raw**(undistort 전) 픽셀이다. POS가 보내는 정의와 동일
(`src/calib.hpp` 규약: "CCTV는 마커 코너를 원본 픽셀 좌표로만 보낸다").
왜곡 보정과 `findHomography` 피팅은 카메라 내부에서 수행한다. 번들의
`coord_mode`는 그대로 `"undistort"`.

서버가 raw 픽셀로 계산하는 폐합오차(§4)는 **mm 환산 불가** — 그 시점엔 새
캘리가 없다. 서버 로그는 픽셀 지표로만 남기고, mm 환산본은 카메라 기록이 정본.

---

## 6. `H_MATRIX` 번들 — 필수 필드

```json
{
  "calib_id": "...", "created_at": "...",
  "image_size": [2592, 1520], "coord_mode": "undistort", "unit": "mm",
  "method": "robot_motion",
  "K": [[...]], "D": [...],
  "H_floor": [[...]], "H_marker": [[...]],
  "marker_height_mm": 160,
  "origin_mm": [0, 0], "canvas_mm": [900, 600],
  "axis": "x_right_y_up"
}
```

- `method`: `"checkerboard"` \| `"robot_motion"` — **탭이 아니라 번들에 실린
  `H_marker`가 실제로 어느 슬롯(측정/파생)에서 왔는지**로 채운다.
- `K`/`D`: **필수.** 카메라는 이 필드가 비어 있으면 **전송을 거부**한다
  (floor 탭과 Odometry 탭 공통 게이트). `coord_mode:"undistort"`인데 `K`/`D`가
  없으면 서버가 왜곡 보정을 조용히 건너뛰고 좌표가 틀어지기 때문
  (`src/calib.hpp`의 `Calib::hasKD` 게이트 참고).
- `H_marker`: 이 방식이 **직접 측정**한 값. 시차 보정(체커보드 방식의
  바닥→마커 변환)을 걸지 않는다 — 이미 마커 평면이다.
- `H_floor`: 카메라가 역산 (`H_marker` + `K` 분해 + 마커/카메라 높이).
  운영자 입력 2종(마커 높이, 카메라 높이 실측)을 쓴다. 상세는
  `REPLY_SERVER_20260812.md` §2-4.
- `origin_mm`/`canvas_mm`: 이번 주행의 사각형 — **세션마다 원점이 로봇 출발
  위치라 바뀐다** (수용된 결정).

서버는 이 번들을 저장·중계할 때 값을 바꾸지 않는다 (mm 보존 규약,
`QT_CCTV_SERVER_CALIBRATION_FORMAT_20260811.md`).

---

## 7. 실패 사유 — 지점 수준 / 세션 수준

| reason | 수준 | 서버 처리 |
|---|---|---|
| `marker_not_found` | 지점 | 그 점 버리고 계속 |
| `not_settled` | 지점 | 그 점 버리고 계속 |
| `unmappable` | 지점 | 그 점 버리고 계속 |
| `no_intrinsics` | 세션 | 즉시 중단 → `CALIB_FAIL` |
| `session_conflict` | 세션 | 즉시 중단 → `CALIB_FAIL` |
| `detect_off` | 세션 | 즉시 중단 (그 렌즈의 검출이 꺼져 있어 프레임이 오지 않는다) |
| `session_refused` | 세션 | 즉시 중단 (카메라가 수집 세션을 열지 못했다) |
| `store_failed` | 세션 | 즉시 중단 (지점 저장 실패 — 슬롯 초과 등) |
| `fit_failed` | 세션 | (카메라가 `CALIB_DONE` 이후 자체 판정, `H_MATRIX` 대신 `CALIB_FAIL`) |
| `too_few_points` | 세션 | 위와 동일 |
| `capture_timeout` | 세션 | 서버가 판정 (§8), 즉시 중단 |
| `invalid_param` | 개시 거절 | 서버가 판정 (§1) — 세션이 시작되지 않는다 |
| `not_owner` | 취소 거절 | 소유자가 아닌 쪽의 `CALIB_CANCEL` (§1-1) — 세션은 그대로 진행 |
| `preempted` | 세션 | 관리자창의 강제 회수 (§1-1). 소유자에게 `CALIB_CANCELLED`가 아니라 이 사유로 간다 |
| `qt_offline` | 세션 | QT 소유 세션에서 Qt 연결이 끊김 — 주인 없는 로봇을 세운다 (§1-1) |

지점 수준 실패가 누적돼 **유효 대응점이 6개 미만**이면 세션 실패로 취급한다
(카메라가 `CALIB_DONE` 처리 시 판정, `too_few_points`로 `CALIB_FAIL`).

---

## 8. 타임아웃 4종 (2026-08-13 갱신)

| 주체 | 파라미터/값 | 대상 | 만료 시 동작 |
|---|---|---|---|
| 서버 | `calib_capture_timeout_ms` = 15000 | 캡처 ack 1건 | `abortOdoCalib("capture_timeout")` — 세션 즉시 중단 (건너뛰지 않는다) |
| 서버 | `calib_odo_timeout_ms` = 300000 | **주행** (START → `CALIB_DONE`) | `abortOdoCalib("timeout")` |
| 서버 | `calib_odo_result_wait_ms` = 60000 | **결과 대기** (`CALIB_DONE` → `H_MATRIX`) | `failCalib("timeout")` — QT 개시 세션에만 있는 구간 |
| 카메라 | 10초 | 정지 판정 1건 | `CALIB_CAPTURE_FAIL{reason:"not_settled"}` |
| 카메라 | 10분 | 마지막 `CALIB_CAPTURE`부터 무응답 | 카메라 세션 폐기, `/status`에 기록. 이후 그 세션으로 오는 `CALIB_CAPTURE`는 `CALIB_CAPTURE_FAIL{reason:"no_session"}`으로 명시 거부 |

`calib_capture_timeout_ms`(15초)는 카메라 자체 정지판정 한도(10초)보다
여유 있게 잡았다 — 카메라가 정상적으로 실패를 보고할 시간을 서버가 먼저
끊지 않기 위해서다.

결과 대기 구간에서는 `abortOdoCalib()`이 아니라 `failCalib()`을 쓴다. 로봇은
이미 서 있고 경로도 비운 뒤라 **세울 대상이 없기 때문**이다 — 그대로 정지
핸드셰이크를 돌리면 오지 않을 `CALIB_STOPPED`를 5초 기다리다 `cancel_failed`로
한 번 더 늘어지고, Qt에는 "정지 확인 실패, 로봇 상태를 직접 확인하세요"라는
엉뚱한 경고가 뜬다.

### 🔴 Qt 개시 세션의 예산 (2026-08-13)

`calib_odo_timeout_ms`에는 **상한 클램프가 없다.** 원래 근거는 "이 방식은 Qt가
트리거하지 않으므로 'Qt 5분보다 짧아야 한다'는 제약이 없다"였는데, §1이 바뀌면서
그 전제가 깨졌다. 그렇다고 파라미터 값 자체를 깎으면 ADMIN 개시 세션까지 같이
짧아진다 — 그쪽은 기다리는 Qt가 없어 깎을 이유가 없다.

그래서 클램프를 **세션 단위**로 옮겼다 (`Router::odoDriveBudgetMs()`):

| 개시자 | 주행 데드라인 |
|---|---|
| ADMIN | `calib_odo_timeout_ms` 그대로 (300초) |
| QT | `min(calib_odo_timeout_ms, kQtCalibWaitCapMs − calib_odo_result_wait_ms)` = **230초** |

`kQtCalibWaitCapMs`(290000)는 Qt 대기 한도 300초에서 한 뼘 뺀 값이다. Qt 클라이언트
쪽 상수라 `params.json`으로 못 바꾸게 코드 상수로 뒀다 — 현장에서 서버만 늘리고
Qt는 그대로인 어긋남을 만들지 않기 위해서다. **Qt팀이 대기 한도를 바꾸면
`params.hpp`의 이 상수도 같이 바꿔야 한다.**

---

## 9. 안전 정지 — 취소·타임아웃이 로봇을 세우는 경로

**정적 앵커 방식과 동일한 핸드셰이크를 쓴다.** `robot_motion` 세션에서
취소·타임아웃이 발생하면:

```
Server → Robot: CALIB_CANCEL   (ack: CALIB_STOPPED)
Server → CCTV:  CALIB_CANCEL   (ack: CALIB_STOPPED)
   → 양쪽 ack가 calib_cancel_ack_ms(5000) 안에 모여야 세션을 닫는다
```

로봇은 R-2 핸들러에서 속도 0 + 노즐 UP + 경로 폐기 + 래치 클리어를 마친 뒤
**100ms 정착 후 `CALIB_STOPPED`를 회신**한다.

`ABORT_DRAW`는 쓰지 않는다 — 그것도 로봇을 세우지만 **ack가 없어서**(수신 즉시
속도 0을 내리는 동기 처리, `DRAW_ABORTED`는 서버가 QT에 보내는 통지지 로봇의
회신이 아니다) "명령을 보냈다"까지만 알 수 있다. 이 규약이 지키려는 것은
**"로봇이 실제로 섰다는 확인"** 이므로(`router_calib.cpp:190`) 받을 수 있는
ack를 버릴 이유가 없다.

양쪽 ack가 시간 안에 안 모이면 `cancel_failed`로 닫는다 — "아마 섰겠지"로
`CALIB_CANCELLED`를 보내지 않는다. 조작자가 로봇을 눈으로 확인하러 가야 한다.

---

## 10. 채널 번호 — 1-based

기존 규약과 동일 (`src/protocol.hpp:358`: "채널 번호는 1부터 시작한다").
`CALIB_CAPTURE`/`CALIB_DONE` 등 신규 메시지도 전부 1-based.

> 2026-08-11에 POS의 ch를 0-based로 내보내 CH1이 통째로 버려진 사고가 있었다.
> 같은 실수를 반복하지 않기 위해 명시한다.

---

## 11. 세션 종료 흐름 정리

```
1. 관리자창|Qt → Server: CALIB_START{method:"robot_motion", ...}
   (Qt 개시면) Server → Qt: CHANNEL_OK, CALIB_STARTED, CALIB_PROGRESS{phase:"driving"}
2. Server → Robot:     PATH{phase:"calib", ops:[11개]}
3. Server → CCTV:      CALIB_CAPTURE × 9 (각 READY/PATH_DONE 직후, 대기 없음)
   CCTV → Server:      CALIB_CAPTURE_OK / CALIB_CAPTURE_FAIL (건마다)
   Server → Robot:     GO (ack 즉시)
   (Qt 개시면) Server → Qt: CALIB_PROGRESS{point_index, valid} (캡처마다)
4. Server → CCTV:      CALIB_DONE{m_mm, n_mm}
   Server: clearPath() — 로봇 몫은 여기서 끝
5. CCTV: 유효 8점(idx 0~7)으로 findHomography + LOO → H_marker 산출
         H_floor 역산, K/D 재활용, 번들 조립
   CCTV → Server:      H_MATRIX{ch, calib:{...}}   (기존 계약, §6 필드)
        또는            CALIB_FAIL{too_few_points|fit_failed}
6. Server: 영속 저장 + Qt에 중계
```

**4에서 세션이 끝나는지는 개시자에 따라 다르다.** 여기가 2026-08-13 변경의 핵심이다.

| | ADMIN 개시 | QT 개시 |
|---|---|---|
| 4에서 | `clearCalib()` — 세션 종료 | `odoAwaitingResult_=true` — **세션 유지** |
| 5의 `H_MATRIX` | `request_id` 없이 중계 (평소 캘리 갱신과 동일) | 종결 응답으로 처리, `request_id` 실림 |
| 5의 `CALIB_FAIL` | "진행 중인 세션 없음"으로 버려짐 | `failCalib()`로 Qt에 전달 |

> **원문(2026-08-12)은 4에서 무조건 `clearCalib()`이었고**, 그 근거가 "Qt는 애초에
> 이 요청을 시작한 적이 없어 대기 중인 것도 없으므로 문제가 되지 않는다"였다.
> Qt 개시를 열면 이 근거가 그대로 무너진다 — 뒤늦게 오는 `H_MATRIX`는 종결로
> 잡히지 않아 `request_id`가 안 실리고, 카메라의 `CALIB_FAIL{too_few_points}`는
> 통째로 버려진다. **둘 다 Qt를 타임아웃까지 대기 화면에 가둔다.** 그래서 QT
> 개시 세션만 4에서 닫지 않고 결과 대기(§8)로 넘긴다.

ADMIN 개시 동작은 원문 그대로다 — 기다리는 쪽이 없으므로 바꿀 이유가 없다.
