# 로봇 오도메트리 호모그래피 — Wire 스펙 (확정본)

- 작성일: 2026-08-12
- 상태: **확정. 서버·CCTV 양쪽 구현의 근거 문서.**
- 이 문서는 규격서다. 왜 이렇게 정했는지는 아래 문서들에 있다 — 여기서는
  되풀이하지 않는다.
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_PLAN_20260811.md`
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_20260812.md`
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_SERVER_20260812.md`
  - `docs/ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_02_20260812.md`

---

## 1. 세션 개시 — 관리자창 전용 (2026-08-12 확정)

Qt는 트리거하지 않는다. v1 범위 밖.

```
관리자창 → Server:  CMD {cmd:"CALIB_START", ch, request_id,
                          method:"robot_motion", m_cm, n_cm, start_corner}
```

| 필드 | 타입 | 필수 | 비고 |
|---|---|---|---|
| `ch` | int | ✅ | 1-based |
| `request_id` | string | ✅ | 서버가 세션 내내 이 값을 정본으로 씀 |
| `method` | string | ✅ | `"robot_motion"` 고정 (없으면 기존 정적 앵커 경로) |
| `m_cm` | number | ✅ | 가로. 기본 90 |
| `n_cm` | number | ✅ | 세로. 기본 60 |
| `start_corner` | string | ✅ | `"bottom_left"` \| `"top_left"` |

서버 → 로봇: `PATH{phase:"calib", ops:[...11개]}` (§3). Qt에는 아무것도 보내지 않는다.

---

## 2. 메시지 5종

```
Server → CCTV   CALIB_CAPTURE       {ch, request_id, point_index, world_xy_mm:[x,y]}
CCTV   → Server CALIB_CAPTURE_OK    {ch, request_id, point_index, pixel_uv:[u,v], spread_px}
CCTV   → Server CALIB_CAPTURE_FAIL  {ch, request_id, point_index, reason}
Server → CCTV   CALIB_DONE          {ch, request_id, m_mm, n_mm}
CCTV   → Server H_MATRIX            {ch, calib:{...}}        (기존 계약, §6)
```

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

### 2-1. 매칭 키 — `ch` + `request_id` + `point_index` 세 개 전부 일치

하나라도 어긋나면 **무시하고 WARN 로그만** 남긴다 (응답하지 않는다 — 그 메시지는
지금 세션의 것이 아니다). 세션 재시작 시 `point_index`가 0으로 리셋되므로
`request_id`만으로는 이전 세션의 늦은 ack와 새 세션의 대기가 같은 인덱스에서
충돌할 수 있다. 세 값 전부를 키로 쓴다.

---

## 3. 로봇 경로 — 11-op, 로봇 펌웨어 변경 없음

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
  2초 정지 대기 후 `READY(0)` — 출발점의 정착 대기가 별도 구현 없이 충족된다.

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
| `fit_failed` | 세션 | (카메라가 `CALIB_DONE` 이후 자체 판정, `H_MATRIX` 대신 `CALIB_FAIL`) |
| `too_few_points` | 세션 | 위와 동일 |
| `capture_timeout` | 세션 | 서버가 판정 (§8), 즉시 중단 |

지점 수준 실패가 누적돼 **유효 대응점이 6개 미만**이면 세션 실패로 취급한다
(카메라가 `CALIB_DONE` 처리 시 판정, `too_few_points`로 `CALIB_FAIL`).

---

## 8. 타임아웃 3종

| 주체 | 파라미터/값 | 대상 | 만료 시 동작 |
|---|---|---|---|
| 서버 | `calib_capture_timeout_ms` = 15000 | 캡처 ack 1건 | `abortOdoCalib("capture_timeout")` — 세션 즉시 중단 (건너뛰지 않는다) |
| 서버 | `calib_odo_timeout_ms` = 300000 | 세션 전체 | `abortOdoCalib("timeout")` |
| 카메라 | 10초 | 정지 판정 1건 | `CALIB_CAPTURE_FAIL{reason:"not_settled"}` |
| 카메라 | 10분 | 마지막 `CALIB_CAPTURE`부터 무응답 | 카메라 세션 폐기, `/status`에 기록. 이후 그 세션으로 오는 `CALIB_CAPTURE`는 `CALIB_CAPTURE_FAIL{reason:"no_session"}`으로 명시 거부 |

`calib_capture_timeout_ms`(15초)는 카메라 자체 정지판정 한도(10초)보다
여유 있게 잡았다 — 카메라가 정상적으로 실패를 보고할 시간을 서버가 먼저
끊지 않기 위해서다.

`calib_odo_timeout_ms`는 기존 `calib_timeout_ms`와 **별개 파라미터**다.
`calib_timeout_ms`는 정적 앵커 방식과 공유되고 그쪽은 지금도 Qt가 트리거하므로
`params.hpp`가 300000 이상을 자동으로 290000까지 깎는 클램프를 걸어둔다("Qt가
5분 안에 스스로 포기하므로 서버가 그보다 짧아야 한다"). 오도메트리 세션은 Qt가
절대 트리거하지 않으므로(§1) 이 제약이 적용되지 않는다 — 그래서 값을 공유하지
않고 새 파라미터를 뒀다.

---

## 9. 안전 정지 — 취소·타임아웃이 로봇을 세우는 경로

**정적 앵커 방식과 다르다.** `robot_motion` 세션에서 취소·타임아웃이 발생하면:

```
Server → Robot: CMD{cmd:"ABORT_DRAW"}   (fire-and-forget, ack 없음)
Server → CCTV:  CALIB_CANCEL             (ack: CALIB_STOPPED)
```

로봇 펌웨어에 `CALIB_*` 핸들러가 없으므로(`Paint_Robot/RaspberryPi/` 확인,
0건), 기존 `CALIB_CANCEL`을 로봇에 보내면 무시되고 로봇이 계속 주행한다.
`ABORT_DRAW`는 로봇이 실제로 이해하는 명령이라 이걸 쓴다.

**`ABORT_DRAW`에는 ack가 없다** — 로봇(`main.cpp:77`)은 수신 즉시 STM32에
속도 0을 내리는 동기 처리라 서버로 돌아오는 확인 메시지가 없다. 정적 앵커
방식의 `CALIB_CANCEL`↔`CALIB_STOPPED`(양쪽 ack 대기)와 의도적으로 다른
규약이다. 로봇의 정지는 fire-and-forget으로 즉시 처리되고, **CCTV의
`CALIB_STOPPED` 하나만** `calib_cancel_ack_ms`(5000) 안에 기다린다. 로봇의
동기 정지가 CCTV 응답보다 항상 먼저 끝나므로 순서 문제는 없다. 타임아웃
안에 `CALIB_STOPPED`가 안 와도 로그만 남기고 세션은 닫는다 — 여기서 갇히면
세션이 영영 안 닫힌다.

---

## 10. 채널 번호 — 1-based

기존 규약과 동일 (`src/protocol.hpp:358`: "채널 번호는 1부터 시작한다").
`CALIB_CAPTURE`/`CALIB_DONE` 등 신규 메시지도 전부 1-based.

> 2026-08-11에 POS의 ch를 0-based로 내보내 CH1이 통째로 버려진 사고가 있었다.
> 같은 실수를 반복하지 않기 위해 명시한다.

---

## 11. 세션 종료 흐름 정리

```
1. 관리자창 → Server: CALIB_START{method:"robot_motion", ...}
2. Server → Robot:     PATH{phase:"calib", ops:[11개]}
3. Server → CCTV:      CALIB_CAPTURE × 9 (각 READY/PATH_DONE 직후, 대기 없음)
   CCTV → Server:      CALIB_CAPTURE_OK / CALIB_CAPTURE_FAIL (건마다)
   Server → Robot:     GO (ack 즉시)
4. Server → CCTV:      CALIB_DONE{m_mm, n_mm}
5. CCTV: 유효 8점(idx 0~7)으로 findHomography + LOO → H_marker 산출
         H_floor 역산, K/D 재활용, 번들 조립
   CCTV → Server:      H_MATRIX{ch, calib:{...}}   (기존 계약, §6 필드)
6. Server: 영속 저장 + Qt에 **즉시** 중계 (`handleHMatrix()`의 `srv_.sendTo("QT",
   outMsg)`는 세션 종결 여부와 무관하게 무조건 실행된다 — `router.cpp:946`).
   단, `request_id`는 실리지 않는다 — 캘리 세션이 4에서 이미 `clearCalib()`으로
   닫혀 `calibActive_`가 false이므로 "종결 응답"(§3.4 계약)으로는 처리되지
   않는다. Qt는 애초에 이 요청을 시작한 적이 없어 대기 중인 것도 없으므로
   문제가 되지 않는다 — 평소 캘리 갱신 중계와 동일하게 받는다.
```

세션은 4에서 서버 관점으로는 끝난다(§1 참고, Qt 대기가 없으므로 종결 응답
불변식이 적용되지 않는다). 5는 카메라의 후속 작업이고, 6은 5의 결과가
`H_MATRIX`로 들어오는 즉시 서버가 처리하는 일반 경로다 — 별도 대기나 지연이
없다.
