# Road-Painter 서버 테스트 가이드

서버(TLS 릴레이 + 경로생성)를 실제 클라이언트 없이 검증하고, Qt 팀이 서버에 붙이기 위한 안내입니다.
메시지 규격 전체는 **[PROTOCOL.md](PROTOCOL.md)** 참고.

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

## 3. Qt 없이 서버 단독 검증 (qt_sim)

Qt 앱이 없어도 **QT 역할 대역**으로 전체 흐름을 검증할 수 있습니다. 서버를 켠 채로 **새 터미널**에서:

```bash
make qt_sim
./qt_sim 127.0.0.1 certs/server.crt      # 서버와 같은 기기면 127.0.0.1
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

**터미널 C — qt_sim** (3번에서 쓴 것): `login` 후 `blueprint` 전송.

기대 동작: 서버가 CCTV의 POS로 로봇 pose를 계산 → qt_sim에 `POSE` 전송, 도면 + pose를 조합 → ROBOT(터미널 B)에 `PATH` 전송. 각 콘솔에 찍히면 **end-to-end 정상**.

> `openssl s_client`는 자체서명 인증서라 접속 시 verify 경고를 찍지만 통신은 정상입니다.

## 5. 확인 포인트 체크리스트

- [ ] qt_sim이 `LOGIN_OK` 받음
- [ ] CCTV `POS` 전송 시 qt_sim에 `POSE {x,y,theta_deg}`가 옴 (서버가 변환한 것)
- [ ] `blueprint` + POS가 모두 있을 때 ROBOT에 `PATH {segments}`가 옴
- [ ] 서버 콘솔에 `[INFO] 경로 생성 완료` 로그

---

# 📱 Qt 쪽 가이드

## 0. 먼저 받을 것

1. 서버 담당에게 **`server.crt`** 파일 (앱에 동봉)
2. **서버 IP** (예: `192.168.0.8`), **포트 9000**
3. **[PROTOCOL.md](PROTOCOL.md)** — 메시지 규격 전체

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
