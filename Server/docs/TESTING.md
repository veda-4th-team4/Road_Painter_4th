# Road-Painter 서버 테스트 가이드

서버(TLS 릴레이 + 경로생성)를 실제 클라이언트 없이 검증하고, Qt 팀이 서버에 붙이기 위한 안내입니다.
메시지 규격 전체는 **[server_PROTOCOL.md](../server_PROTOCOL.md)** 참고.

- 전송: **TCP + TLS, 포트 9000**
- 프레이밍: JSON 한 줄 + `\n` (JSON Lines)
- 메시지 형식: `{"type":"...","seq":n,"payload":{...}}`
- 등록: 접속 후 10초 안에 `HELLO`(role 지정) 필수

> ⚠️ 기존 팀원 프로토타입(`jsn_server.c` 등)의 **포트 25000 · 평문 TCP · `CONN_INIT`/`device_id` · flat JSON은 쓰지 않습니다.** 아래 규약(TLS / 9000 / `HELLO`+role / 중첩 payload)으로 통일합니다.

---

# 🖥️ 서버 쪽 가이드 (서버 RPi에서)

## 1. 최초 1회 준비

```bash
cd ~/Road_Painter_4th/Server

# 패키지 (한 번만)
sudo apt install g++ make libssl-dev nlohmann-json3-dev

# TLS 인증서 생성 — 반드시 "서버 RPi의 실제 LAN IP"를 넣을 것 (ip addr 로 확인)
./gen_cert.sh 192.168.0.8
```

`certs/server.crt`가 생기면 **이 파일을 Qt·CCTV·로봇 팀에 전달**합니다 (신뢰 CA로 등록해야 TLS가 붙음).
IP를 바꾸면 인증서를 다시 만들어 재배포해야 합니다 (SAN에 IP가 박히기 때문).

### server.crt 배포 방법

`server.crt`는 **공개 인증서**라 어떤 채널로 보내도 안전합니다. 반대로 **`server.key`는 절대 서버 밖으로 내보내지 마세요**.

- **scp (같은 LAN, 권장)** — 서버에서 밀어넣기:
  ```bash
  scp certs/server.crt 팀원계정@192.168.0.20:~/
  ```
  또는 팀원 PC(윈도우 PowerShell 포함)에서 당겨오기:
  ```bash
  scp team4@192.168.0.8:~/Road_Painter_4th/Server/certs/server.crt .
  ```
- **HTTP 임시 공유 (여러 명 한꺼번에)** — key 노출 막으려 crt만 따로 열기:
  ```bash
  mkdir -p /tmp/share && cp certs/server.crt /tmp/share && cd /tmp/share && python3 -m http.server 8000
  # 팀원: curl -O http://192.168.0.8:8000/server.crt  (다 받으면 Ctrl+C)
  ```
- **USB / 메신저 / 이메일** — 공개 텍스트 파일이라 그대로 보내도 무방. 압축해 보낼 땐 zip 안에 `.crt` 하나만 들었는지 확인 (`.key` 포함 금지).

## 2. 빌드 & 실행

```bash
make            # server 빌드
./server        # 포그라운드 실행, 로그가 콘솔에 출력됨
```

`[INFO] Road-Painter TLS 서버 시작 0.0.0.0:9000` 이 뜨면 대기 상태.

## 2-1. 테스트 계정 만들기 (seed_user.py)

`config/users.json`은 비밀번호 해시가 들어있어 git에 올라가지 않습니다. 테스트 계정은 스크립트로 만드세요.

```bash
python3 tools/seed_user.py
```

기본값으로 **`test` / `1234`** 계정이 만들어집니다. 예시 호모그래피(캘리브레이션)와 **카메라 IP `192.168.0.9`** 가 함께 들어가서, 로그인하면 `LOGIN_OK`로 `calib`과 `cam_ip`가 바로 내려옵니다(top-view + RTSP 영상 둘 다 테스트 가능).

```bash
python3 tools/seed_user.py --id u2 --pw pw2 --no-calib   # 캘리 없는 계정
python3 tools/seed_user.py --id test --pw 1234 --force   # 기존 계정 덮어쓰기
python3 tools/seed_user.py --cam-ip 192.168.0.31         # 카메라 IP 바꿔서
python3 tools/seed_user.py --h-json '[[...],[...],[...]]' # 직접 준 H(mm) 사용
```

만든 계정으로 **관리자 창(`http://<서버IP>:8083`)에 로그인**하면 카메라 캘리브레이션·로봇 제어·로그 모니터가 열립니다. 로그인 전에는 전부 `/login`으로 리다이렉트됩니다 — 캘리 결과가 로그인된 계정에 저장되는 구조라 순서를 강제한 것입니다.

- 기존 계정은 보존됩니다(같은 id는 `--force` 없이는 안 덮어씀).
- ⚠️ `path_test`를 로그인 상태로 돌리면 스냅샷의 H가 **그 계정의 캘리브레이션을 덮어씁니다**. 실측 H로 되돌리려면 `--force`로 다시 시드하세요.
- ⚠️ **`--h-json`에 넣는 H는 CCTV가 주는 그대로의 mm 기준**입니다. 스크립트가 `÷1000`해서 미터로 저장합니다 — 로그인 경로(`getCalib`)에는 서버의 mm→m 정규화가 없어서, 파일에는 이미 미터인 값이 들어있어야 하기 때문입니다.
- ⚠️ **서버가 실행 중이면 재시작해야 반영됩니다** (`UserStore`는 기동 시 파일을 1회만 읽음).

## 3. Qt 없이 서버 단독 검증 (qt_sim)

Qt 앱이 없어도 **QT 역할 대역**으로 전체 흐름을 검증할 수 있습니다. 서버를 켠 채로 **새 터미널**에서:

```bash
make qt_sim
./tools/qt_sim 127.0.0.1 certs/server.crt      # 서버와 같은 기기면 127.0.0.1
```

qt_sim 콘솔에서 순서대로:

```
register test 1234      → REGISTER_OK 확인
login test 1234         → LOGIN_OK, calib:null (아직 캘리브레이션 없음)
blueprint               → 테스트 도면 [0,0]→[2,0]→[2,1] 전송
cmd calib               → CALIB_START 전달 확인
```

서버 콘솔과 qt_sim 콘솔 로그가 서로 맞물리면 QT ↔ 서버 경로는 정상.

## 4. 전체 파이프라인 검증 (CCTV·로봇 대역까지)

POSE 계산·PATH 생성까지 보려면 CCTV(POS)와 ROBOT을 흉내내야 합니다. `openssl`로 각 역할을 띄웁니다. **터미널 3개**를 쓰세요.

**터미널 A — CCTV 역할** (캘리브레이션 올리고 마커 위치 전송):
```bash
openssl s_client -quiet -connect 127.0.0.1:9000
# 접속되면 아래 JSON을 한 줄씩 붙여넣기 (각 줄 끝 Enter)
{"type":"HELLO","seq":1,"payload":{"role":"CCTV"}}
{"type":"H_MATRIX","seq":2,"payload":{"calib":{"version":1,"K":[[800,0,320],[0,800,240],[0,0,1]],"D":[0,0,0,0,0],"H_floor":[[0.01,0,0],[0,0.01,0],[0,0,1]],"H_marker":[[0.01,0,0],[0,0.01,0],[0,0,1]],"marker_height_m":0.25}}}
{"type":"POS","seq":3,"payload":{"corners":[[100,200],[200,200],[200,100],[100,100]]}}
```

**터미널 B — ROBOT 역할** (PATH 받는 쪽):
```bash
openssl s_client -quiet -connect 127.0.0.1:9000
{"type":"HELLO","seq":1,"payload":{"role":"ROBOT"}}
{"type":"STATUS","seq":2,"payload":{"state":"IDLE","painting":false}}
```

**터미널 C — qt_sim** (3번에서 쓴 것): `login` 후 `blueprint` 전송, 그 다음 **`cmd start_draw`** 전송 (2026-07-27부터: `blueprint`만으로는 로봇이 안 움직임 — 저장만 됨).

기대 동작: 서버가 CCTV의 POS로 로봇 pose를 계산 → qt_sim에 `POSE` 전송. `blueprint`는 저장만 되고, **`cmd start_draw`를 보내야** 도면 + pose를 조합해 ROBOT(터미널 B)에 접근 `PATH`가 전송된다. 각 콘솔에 찍히면 **end-to-end 정상**.

> `openssl s_client`는 자체서명 인증서라 접속 시 verify 경고를 찍지만 통신은 정상입니다.

> 터미널 B(ROBOT 역할)가 `PATH_DONE`을 보내지 않는 단순 openssl 세션이라면, 접근 PATH 전송까지만 확인되고 도색 단계로는 자동 진행되지 않습니다(정상 — 실제 로봇이 `PATH_DONE`을 보내야 서버가 이어서 도색 PATH를 보냅니다). 도색까지 보려면 터미널 B에서 접근 PATH를 받은 뒤 `{"type":"PATH_DONE","seq":3,"payload":{"phase":"approach"}}`를 직접 붙여넣으세요.

## 5. 확인 포인트 체크리스트

- [ ] qt_sim이 `LOGIN_OK` 받음
- [ ] CCTV `POS` 전송 시 qt_sim에 `POSE {x,y,theta_deg}`가 옴 (서버가 변환한 것)
- [ ] `blueprint` 전송만으로는 ROBOT에 아무것도 안 옴 (저장만 됨 확인)
- [ ] `blueprint` 후 `cmd start_draw`를 보내면 ROBOT에 `PATH {phase:"approach", segments}`가 옴
- [ ] 서버 콘솔에 `[INFO] 1단계 접근 경로 전송` 로그
- [ ] (선택) ROBOT 역할에서 `PATH_DONE{phase:"approach"}`를 보내면 ROBOT에 `PATH {phase:"draw"}`가 이어서 옴 + 서버 콘솔에 `[INFO] 2단계 도색 경로 전송`
- [ ] (선택) 이어서 `PATH_DONE{phase:"draw"}`를 보내면 qt_sim에 `DRAW_DONE`이 옴

## 6. 최초 1회 경로생성 테스트 (path_test) — 서버 RPi ↔ 로봇 RPi 중점

CCTV가 딱 한 번 만들어 저장해둔 자료(호모그래피 + 로봇 시작 4코너)를 읽어 서버에
주입하고, 서버가 로봇을 **(0,0)으로 보내는 approach PATH(TURN/MOVE)** 를 한 번
생성하는지 검증한다. 실시간 피드백(DRIFT/이탈 재계획)은 자연히 빠진다(POS 1회 +
approach 단계는 재계획 안 함). **경로생성(각도·거리 계산) 로직은 서버 원본 그대로**이고,
도구는 입력 주입 + 응답 로깅만 한다 — CCTV/QT 대역을 한 프로세스가 대신 물려준다.

```bash
make path_test
./tools/path_test <서버IP> [server.crt경로] [스냅샷.json]
#  기본 crt    = ../certs/server.crt (Server/에서 실행 시 certs/server.crt 지정)
#  기본 스냅샷 = tools/sample_snapshot.json  (실측은 tools/cctv_snapshot.json)
```

- **스냅샷 파일**(`tools/cctv_snapshot.json`) = CCTV가 준 자료를 담는 곳:
  `H`(pixel→world **mm** 호모그래피) + `corners`(로봇 마커 4코너 원본 픽셀).
  CCTV 출력 포맷이 바뀌면 도구의 `loadSnapshot()` **한 함수만** 고치면 된다.
- **단위**: H는 **mm 기준**으로 그대로 담는다. 서버도 캘리 번들을 **mm 그대로**
  저장·중계하고, ÷1000은 내부 계산용 사본에서만 하므로(`calibFromJson`) 도구는 원본
  mm를 손대지 않고 올린다(여기서 환산하면 이중 스케일). `BLUEPRINT.points`와 도구가
  주입하는 도착 `POS{x,y}`는 서버 내부 단위인 **미터**로 보낸다(도구의
  `kTargetPointsMm`은 mm로 적고 전송 시 ÷1000).
  단위 규약 회귀 테스트: `make calib_unit_test && ./tools/calib_unit_test`
- ⚠️ **로봇은 role당 1개**: 서버는 ROBOT 재접속 시 기존 세션을 교체한다. 실제 로봇
  RPi로 테스트할 땐 다른 로봇 대역을 띄우지 말 것.
- **2단계 진행**: 접근(→ points[0]) 후 로봇이 멈추면 도구 콘솔에서 **Enter** → 도구가
  도착 POS 주입 + START_DRAW 전송 → 도색 PATH(→ points[1..]) 진행. (실시간 CCTV가
  없어 서버가 도착을 모르므로, 알려진 접근점을 POS로 주입해 2단계 경로를 맞춘다.)

기대 결과: 서버 콘솔 `[INFO] 1단계 접근 경로 전송 (N 세그먼트)`, 로봇 RPi 콘솔에
`PATH received`(approach). path_test 콘솔엔 QT 대역이 받은 `POSE{x,y,theta_deg}`가
찍힌다(로봇 미접속이면 `DRAW_FAIL{robot_offline}`).

---

# 📱 Qt 쪽 가이드

## 0. 먼저 받을 것

1. 서버 담당에게 **`server.crt`** 파일 (앱에 동봉)
2. **서버 IP** (예: `192.168.0.8`), **포트 9000**
3. **[server_PROTOCOL.md](../server_PROTOCOL.md)** — 메시지 규격 전체

## 1. 접속 골격 (QSslSocket)

```cpp
socket = new QSslSocket(this);

// server.crt 를 신뢰 CA로 등록 (자체서명 핀닝)
QSslConfiguration cfg = socket->sslConfiguration();
cfg.setCaCertificates(QSslCertificate::fromPath("server.crt"));
socket->setSslConfiguration(cfg);
socket->setPeerVerifyMode(QSslSocket::VerifyPeer);

// 암호화 완료되면 HELLO 먼저 (접속 후 10초 안에 안 보내면 서버가 끊음)
connect(socket, &QSslSocket::encrypted, this, [this]{
    sendMsg("HELLO", QJsonObject{{"role","QT"}});
});

// 수신: \n 로 프레이밍해서 한 줄씩 파싱 (TCP는 쪼개져 오므로 버퍼링 필수)
connect(socket, &QSslSocket::readyRead, this, [this]{
    rxBuf += socket->readAll();
    int nl;
    while ((nl = rxBuf.indexOf('\n')) >= 0) {
        QByteArray line = rxBuf.left(nl);
        rxBuf.remove(0, nl + 1);
        QJsonObject msg = QJsonDocument::fromJson(line).object();
        handleMsg(msg["type"].toString(), msg["payload"].toObject());
    }
});

socket->connectToHostEncrypted(serverIp, 9000);
```

```cpp
// 송신 헬퍼 — 모든 메시지는 {type, seq, payload} + 끝에 \n
void sendMsg(const QString& type, const QJsonObject& payload) {
    QJsonObject m{{"type",type}, {"seq",++seq}, {"payload",payload}};
    socket->write(QJsonDocument(m).toJson(QJsonDocument::Compact) + "\n");
}
```

## 2. Qt가 보내는 메시지 (QT → 서버)

| 언제 | 코드 |
|---|---|
| 접속 직후 | `sendMsg("HELLO", {{"role","QT"}})` |
| 회원가입 | `sendMsg("REGISTER", {{"id",id},{"pw",pw}})` |
| 로그인 | `sendMsg("LOGIN", {{"id",id},{"pw",pw}})` |
| 비상정지/재개 | `sendMsg("CMD", {{"cmd","ESTOP"}})` / `"RESUME"` |
| 캘리브레이션 시작 | `sendMsg("CMD", {{"cmd","CALIB_START"}})` |
| 도면 전송 | `sendMsg("BLUEPRINT", {{"points", pts}})` |

`points`는 **바닥 미터 좌표** 배열 `[[x,y],...]` — top-view 위에 그린 픽셀을 축척 S(px/m)로 나눈 값. 이 변환은 Qt가 끝내서 보냅니다.

## 3. Qt가 받는 메시지 (서버 → QT) — `handleMsg`에서 분기

| type | payload | 할 일 |
|---|---|---|
| `ACK` | `{msg}` | 접속 확인 |
| `LOGIN_OK` | `{id, calib}` | `calib`가 null이면 → 캘리브레이션 유도. 아니면 `calib.H_floor`(+K,D)로 top-view 생성 |
| `LOGIN_FAIL` / `REGISTER_FAIL` | `{reason}` | 에러 표시 |
| `REGISTER_OK` | `{id}` | 가입 완료 |
| `H_MATRIX` | `{calib}` | top-view를 새 `H_floor`로 재생성 |
| `STATUS` | `{state, painting}` | 로봇 상태 UI (`IDLE`/`MOVING`/`ESTOPPED`/`ERROR`) |
| `POSE` | `{x, y, theta_deg}` | **top-view 위 로봇 아이콘 표시** (× S 하면 화면 픽셀). 로봇 위치는 이걸 씀 |
| `POS` | `{corners}` | 원본 CCTV 픽셀이라 화면엔 못 씀 — 무시/디버그용 |

## 4. Qt 단독 테스트 순서

서버를 켠 상태에서 Qt를 이 순서로 붙여보며 하나씩 확인:

1. **접속만** → 서버 콘솔에 `[접속] QT` 뜨는지
2. **HELLO** → `ACK` 수신되는지
3. **REGISTER → LOGIN** → `LOGIN_OK` 수신, `calib:null` 확인
4. **BLUEPRINT**(임시로 점 하드코딩) → 서버 콘솔에 `도면 수신` 로그
5. CCTV 대역(서버 쪽 가이드 4번의 openssl)까지 붙이면 → Qt에 `POSE` 들어오는지

## 5. 안 붙을 때 흔한 원인

- **핸드셰이크 실패** → server.crt 미등록, 또는 인증서 SAN의 IP와 접속 IP 불일치 (서버가 IP 바꿔 재발급했는지 확인)
- **10초 뒤 서버가 끊음** → `encrypted` 시그널에서 HELLO를 안 보냄
- **메시지가 씹힘** → `\n` 프레이밍 없이 `readAll()` 한 번에 파싱함 (TCP는 두 메시지가 붙거나 한 메시지가 쪼개져 옴 → 반드시 버퍼링)
- **포트 틀림** → 25000 아니고 **9000**
