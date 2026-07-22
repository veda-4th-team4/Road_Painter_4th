# 관리자 창(Admin Console) 계획

CCTV팀이 만든 `cctv_calibration_manager`(카메라 캘리브레이션 웹 도구)를 가져와,
**서버가 띄우는 통합 "관리자 창"** 으로 확장한다. 카메라 설치 기사가 설치 시
캘리브레이션까지 수행한다는 가정 하에, 어려운 캘리브레이션·로봇 점검을 이 창에서 처리한다.

원본은 `~/cctv_calibration_manager` (CCTV팀 관리). 여기(`Server/admin_console/`)는
그걸 복사해 서버 파트에서 살을 붙인 사본이다. **기존 카메라 기능은 최대한 유지**하고,
필요 없어지면 나중에 뺀다.

## 통합 방식: A안 — 서버에 ADMIN role 추가

서버(C++ TLS, 9000)는 QT/ROBOT/CCTV 세 role만 중계한다. 관리자 창은 여기에
**`ADMIN`** 이라는 네 번째 role로 접속해서:

1. **로그 엿듣기(tap)**: 서버가 중계하는 모든 메시지의 사본을 ADMIN에게도 보낸다.
   → 관리자 창이 QT↔ROBOT↔CCTV 트래픽을 실시간으로 본다.
2. **로봇 제어**: ADMIN이 보낸 CMD/PATH를 서버가 ROBOT에게 전달한다.

기존 카메라 직결(포트 6000)은 그대로 두고, **서버(9000)로 가는 TLS 연결만 하나 추가**한다.

```
카메라 ──TCP(6000)──▶ [admin_console] ──HTTP(8081)──▶ 브라우저
                          │
                          └──TLS(9000, role=ADMIN)──▶ [C++ 서버] ── 로그 tap + 로봇 제어
```

## 관리자 창 구성(탭 3개)

### ① 카메라 캘리브레이션 — 이미 있음(유지)
K/D 내부 파라미터, 호모그래피(바닥 매핑), LDC 왜곡 검사, 스냅샷.
- (선택) 캘리 완료 시 결과 번들 {K,D,H_floor,H_marker}를 서버 `H_MATRIX`로 자동 전송.

### ② 로그 모니터 — 신규
서버가 tap으로 보내주는 `TAP` 메시지를 대시보드에 실시간 표시.
시각 / 방향(IN=클라이언트→서버, OUT=서버→클라이언트) / peer / type / payload 요약.
web_gui의 기존 SSE(`/events`, `broadcast()`) 틀을 재활용.

### ③ 로봇 제어 — 신규
버튼으로 서버에 CMD/PATH 전송:
- 비상정지/재개: `CMD ESTOP` / `CMD RESUME`
- 수동 조작: `CMD FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT/STOP`
- 캘리 시작: `CMD CALIB_START`
- 테스트 경로: `PATH {segments:[...]}`
- 로봇 상태 표시: tap으로 들어오는 `STATUS`(state/painting) 표시.

## 서버 프로토콜 확장 (protocol.hpp 참고)

- HELLO에 `role:"ADMIN"` 허용.
- 서버 → ADMIN: `TAP payload:{"dir":"IN"|"OUT","peer":"QT"|"ROBOT"|"CCTV","msg":{원본메시지}}`
  - dir=IN: peer가 서버로 보낸 것. dir=OUT: 서버가 peer에게 보낸 것.
  - ADMIN 자신과 오간 메시지는 tap하지 않는다(무한 루프 방지).
- ADMIN → 서버: `CMD`/`PATH` → ROBOT에게 전달(CALIB_START는 CCTV에도).
  - 관리자는 점검·설치용이므로 경로 실행 중이어도 차단하지 않고 항상 통과(QT의 A안 차단과 다름).

## 구현 순서
1. [x] 폴더 복사 (`Server/admin_console/`)
2. [x] 서버: ADMIN role + tap + fromAdmin (protocol/tls_server/router) — 빌드+E2E 검증 완료
3. [x] web_gui: 서버(9000) ADMIN TLS 클라이언트 연결(`server_link_loop`) + TAP 로그를 SSE로 흘리기
4. [x] web_gui: 로그 모니터 — 현재는 기존 SSE 피드에 `[tap]` 라인으로 인라인 표시
       (별도 필터/전용 탭은 추후. 지금도 대시보드/` /robot` 로그에서 실시간으로 보임)
5. [x] web_gui: 로봇 제어 패널 — 독립 페이지 `/robot` (ESTOP/RESUME/조이스틱/CALIB_START)
       + 실시간 상태 배너(서버 연결 / 로봇 state / 도색 / pose) — tap 로그를 클라이언트에서 파싱
6. [x] 캘리 결과 → 서버 H_MATRIX 자동 전송 (calib_cache_k/h → push_calib_to_server)
       서버는 CCTV/ADMIN 공용 handleHMatrix로 처리(저장+Qt 중계). E2E 검증 완료.
7. [x] 탭 바 통합(가벼움) — `/`, `/robot`, `/logs` 상단 공용 탭바(tabbar 주입). 기존 대시보드 원형 유지.
       로그 모니터 `/logs`는 대상 필터(ROBOT/QT/CCTV) 지원.

## 남은 것(선택, 미착수)
- 로그인 붙여 캘리 결과를 사용자별 영속 저장(현재는 로그인 없으면 세션에만 유지)
- 메인 카메라 대시보드 안에 로봇/로그를 SPA식으로 완전 통합(현재는 페이지 전환식)
- QT에 "명령 거절" 응답 규약(현재 차단/드롭은 서버 로그로만 표시)

## 실행 방법
```bash
# 1) 서버 (별도 터미널)
cd Server && make && ./server
# 2) 관리자 창
cd Server/admin_console && python3 web_gui.py
#    브라우저: http://<파이IP>:8081        (카메라 캘리브레이션 대시보드)
#             http://<파이IP>:8081/robot  (로봇 제어 + 서버 로그)
# 서버가 다른 호스트면: RP_SERVER_HOST=x.x.x.x python3 web_gui.py
```
