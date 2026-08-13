# CCTV팀 통지 — 서버 구현 완료, 착수 가능

- 작성일: 2026-08-12
- 수신: CCTV팀 (ArucoPosePNM)
- 발신: 서버팀
- 대상: [`ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_02_20260812.md`](ROBOT_ODOMETRY_HOMOGRAPHY_REPLY_CCTV_02_20260812.md) (2차 회신)
- 상태: **§3 확정 요청 5건 전부 반영 완료 + 변경/추가 통지 3건**

---

## 0. 요약

**2차 회신 §3의 5건을 전부 제안대로 반영해 서버 구현을 마쳤습니다.** wire 스펙
([`..._WIRE_20260812.md`](ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md))이 확정본이고,
그대로 코딩하시면 됩니다. §5의 A~H 착수에 막히는 항목 없습니다.

아래 **§2의 3건만 2차 회신 시점과 달라졌으니** 확인 부탁드립니다.

---

## 1. §3 확정 요청 5건 — 반영 결과

| # | 요청 | 반영 |
|---|---|---|
| 3-1 | `world_xy_mm` = 마커 중심 | ✅ 필드 정의 변경. 서버가 `marker_offset_m`(기본 0.0)로 헤딩별 보정 후 전송 — **카메라는 무관** |
| 3-2 | 매칭 키에 `request_id` 포함 | ✅ `ch` + `request_id` + `point_index` 3개 전부 일치 검사. 지적하신 세션 재시작 구멍이 맞았습니다 |
| 3-3 | 부분 실패 시 진행, 6점 하한 | ✅ 채택. 사유를 2층으로 나눴습니다 — **§2-1 참고** |
| 3-4 | 카메라 10분 타임아웃 + `no_session` | ✅ 그대로 수용 |
| 3-5 | `method` 값 | ✅ 확정. "탭이 아니라 실린 `H_marker`의 슬롯 기준"이 제 요청보다 정확합니다 |

서버팀 요청 3건(`method` 필드 / `K`·`D` 게이트 / `K` stale)도 전부 수용해주셔서
감사합니다. `H_floor` 역산이 `K` 재캘리 시 stale이 되는 건 §6-4에 명시해뒀습니다.

---

## 2. 2차 회신 이후 달라진 것 3건

### 2-1. 실패 사유를 **세션 수준 / 지점 수준**으로 나눴습니다

`CALIB_CAPTURE_FAIL{reason}`을 받았을 때 서버 동작이 갈립니다:

| 수준 | reason | 서버 동작 |
|---|---|---|
| **지점** | `marker_not_found` / `not_settled` / `unmappable` | 그 점만 버리고 **주행 계속** (idx 8까지) |
| **세션** | `no_intrinsics` / `session_conflict` | **즉시 중단** — 로봇 정지 + `CALIB_CANCEL` |

§1-2 표의 사유를 그대로 쓰되, "계속 달려봐야 의미 없는 것"(내부 파라미터 없음,
다른 세션 충돌)만 세션 수준으로 올렸습니다. `fit_failed`/`too_few_points`는
`CALIB_DONE` 이후 카메라 자체 판정이라 서버는 관여하지 않습니다.

**이견 있으시면 알려주세요** — 서버 쪽 한 줄 수정입니다.

### 2-2. `CALIB_START`를 CCTV에도 중계합니다 (§5-B 요청 반영)

2차 회신 §5-B의 "`CALIB_START`가 세션 시작점이 됨"을 반영했습니다. 서버가
세션 개시 시 **ROBOT과 CCTV 양쪽에 원본을 중계**합니다:

```
관리자창 → Server:  CMD{cmd:"CALIB_START", ch, request_id, method:"robot_motion",
                        m_cm, n_cm, start_corner}
Server  → CCTV:     CMD{cmd:"SELECT_CHANNEL", ch}   ← 먼저
Server  → CCTV:     위 CALIB_START 원본 그대로
Server  → ROBOT:    위 CALIB_START 원본 그대로 + PATH{phase:"calib"}
```

> 초안에는 "CCTV에는 `SELECT_CHANNEL`만 보낸다"고 되어 있었는데, 그러면 카메라가
> 세션 없이 `CALIB_CAPTURE`를 받게 됩니다. 로봇 쪽에서도 같은 중계가 노즐 안전
> (R-1 핸들러가 `auto_nozzle=0`으로 강제 UP)에 필요해서 함께 해결됐습니다.

`m_cm`/`n_cm`은 이 메시지에 실려 있으니 필요하시면 여기서 받으셔도 됩니다
(`CALIB_DONE`의 `m_mm`/`n_mm`은 그대로 유지 — §2-5 요청대로).

### 2-3. 세션 타임아웃이 별개 파라미터가 됐습니다

`calib_odo_timeout_ms = 300000`(5분) 신설. 기존 `calib_timeout_ms`(180초)는
정적 앵커 방식과 공유되는 값이라 건드리지 않았습니다 — 그쪽은 Qt가 여전히
트리거하고 "Qt 5분 한도보다 짧아야 한다"는 제약이 살아 있기 때문입니다.

**카메라 쪽 영향 없습니다.** 참고용입니다.

---

## 3. 서버 구현 현황

| 항목 | 상태 |
|---|---|
| `CALIB_CAPTURE` 전송 (9점, `world_xy_mm` 계산 포함) | ✅ |
| `CALIB_CAPTURE_OK` / `CALIB_CAPTURE_FAIL` 수신 + 3키 매칭 | ✅ |
| `CALIB_DONE{m_mm, n_mm}` 전송 | ✅ |
| 캡처 타임아웃 15초 / 세션 타임아웃 300초 | ✅ |
| 안전 정지 (`CALIB_CANCEL` → 양쪽 `CALIB_STOPPED` 대기) | ✅ |
| 폐합오차 픽셀 로깅 (idx 0 vs idx 8) | ✅ |
| `coord_mode=undistort`인데 K/D 없으면 WARN | ✅ |
| 회귀 테스트 | ✅ 33항목 통과, 기존 캘리 회귀 없음 |

**통합 시험은 카메라 구현(§5 A~H) 완료 후**입니다. 실제 로봇·카메라를 붙인
시험은 아직 하지 않았습니다.

---

## 4. 확인 부탁

1. **§2-1의 실패 사유 2층 구분**에 이견 없으신지
2. `CALIB_CAPTURE_OK`의 `spread_px`는 **선택 필드**로 두겠습니다 — 없어도
   서버는 정상 동작하고 로그에 0으로 찍힙니다. 넣어주시면 잔차 대조에 씁니다
3. §4의 사각형 크기 제안(1,800×1,200까지 중앙 85% 안)은 **첫 주행 900×600 유지**
   후 폐합오차를 보고 결정하기로 했습니다 — 계산 감사합니다

---

## 5. 참고 문서

| 문서 | 용도 |
|---|---|
| [`ROBOT_ODOMETRY_HOMOGRAPHY.md`](ROBOT_ODOMETRY_HOMOGRAPHY.md) | **정본** — 설계 전체, 검증 한계(§8) 포함 |
| [`ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md`](ROBOT_ODOMETRY_HOMOGRAPHY_WIRE_20260812.md) | **코딩 근거** — 메시지 형식, 좌표표, 타임아웃 |
| [`ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_ROBOT_20260812.md`](ROBOT_ODOMETRY_HOMOGRAPHY_REQUEST_ROBOT_20260812.md) | 로봇팀 요청 (1m 실측 등 — §4-1 두 방식 비교의 전제) |
