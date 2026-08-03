# RTSP 4채널 패스스루 중계

카메라(PNM-C16083RVQ, 4채널 멀티디렉셔널) → **이 서버** → Qt 클라이언트로 영상을
넘기는 중계. MediaMTX 를 써서 **디코드·재인코딩 없이 그대로 재발행**한다.

```
카메라 4채널 ──RTSP(TCP)──▶ 서버:8554 ──RTSP(TCP)──▶ Qt (2x2)
                            /ch1 /ch2 /ch3 /ch4
```

## ⚠️ 기존 구조는 그대로 둔다

PNM-C16083RVQ 는 **아직 시도 단계**다. 기존 PNO-A9081R 단일 채널 직결 경로
(Qt → 카메라, `LOGIN_OK.cam_ip` 로 URL 조립)는 손대지 않는다 — 언제든 되돌아갈 수
있어야 한다.

그래서 이 중계는 **완전히 선택적**이다:

- 별도 프로세스다. `Server/start.sh` 나 `Makefile` 어디에서도 자동 기동하지 않는다.
  이 디렉터리의 `start.sh` 를 손으로 띄울 때만 존재한다.
- 포트가 안 겹친다 (중계 8554 / 127.0.0.1:9997, 기존 9000 · 6100 · 8083 · 6101).
- 중계를 안 띄우면 서버·카메라·Qt 어느 쪽에도 영향이 없다.
- 되돌리려면 프로세스만 죽이면 된다: `pkill -f relay/bin/mediamtx`

Qt 쪽도 같은 원칙이어야 한다 — 중계 주소는 **추가 설정으로만** 들어가고, 설정이
없으면 지금처럼 카메라에 직결한다.

## 왜 재인코딩을 안 하나

이 서버는 라즈베리파이(aarch64 4코어)다. 4채널 4MP H.265 를 디코드해서 다시
인코딩하는 것은 성능상 불가능하다. MediaMTX 는 받은 패킷을 그대로 넘기므로
(`ffmpeg -c copy` 와 같은 의미) **화질 손실이 0** 이고 CPU 도 거의 안 쓴다.
대신 중계 단을 하나 더 타는 만큼 지연은 직결보다 늘어난다 — 이건 구조상 피할 수 없다.

## ⚠️ 알아둘 제약 두 가지

1. **유선이 있는데 카메라 트래픽은 무선으로 나간다.** 이 서버는 eth0(192.168.0.2,
   100Mbps full-duplex)와 wlan0(192.168.0.8)이 **같은 192.168.0.0/24 에 동시에**
   올라와 있다. 그런데 wlan0 의 서브넷 경로에 metric 이 없고(=0) eth0 은 100 이라,
   커널이 wlan0 을 고른다:

   ```
   $ ip route get 192.168.0.13
   192.168.0.13 dev wlan0 src 192.168.0.8      ← 무선으로 나감
   ```

   중계는 같은 라디오로 수신과 송신을 동시에 한다 — 4채널 메인스트림이면 왕복
   60Mbps 이상이 한 무선 링크에 몰린다. **eth0 로 넘기면** 100M full-duplex 라
   송·수신이 각자 방향을 쓰므로 여유가 생긴다. 끊김이나 프레임 드랍이 보이면
   화질 설정을 만지기 전에 이 경로부터 확인할 것.
2. **채널별 RTSP 경로를 짐작해서 넣지 말 것.** 틀린 URL 로 반복 접속하면 Hanwha
   카메라가 계정을 잠근다 (`RTSP/1.0 490 Account Blocked`). 반드시
   `probe_onvif.py` 로 카메라가 직접 알려주는 값을 받아 쓴다.

## 설치와 기동

```bash
cd Server/relay

# 1) MediaMTX 바이너리 받기 (bin/ 은 gitignore — 60MB)
./install.sh

# 2) 설정 파일 만들기
cp cameras.env.example cameras.env

# 3) 카메라에게 채널별 RTSP 주소 물어보기
python3 probe_onvif.py --host <카메라IP> --user admin --password '<비번>' --env

# 4) 출력된 MTX_PATHS_CH1..CH4_SOURCE 를 cameras.env 에 붙여넣기
#    (이름·해상도를 보고 채널 순서가 맞는지 확인할 것)

# 5) 기동
./start.sh        # 포그라운드 (로그 보며 확인)
./start.sh -d     # 백그라운드 + relay.log
```

기동하면 Qt 는 이 주소들을 본다:

```
rtsp://<서버IP>:8554/ch1
rtsp://<서버IP>:8554/ch2
rtsp://<서버IP>:8554/ch3
rtsp://<서버IP>:8554/ch4
```

## 파일

| 파일 | 역할 | 커밋 |
|---|---|---|
| `mediamtx.yml` | 중계 설정. 카메라 주소·계정 없음 | O |
| `cameras.env.example` | 카메라 접속 정보 템플릿 | O |
| `cameras.env` | 실제 주소·계정 | **X** (gitignore) |
| `probe_onvif.py` | ONVIF 로 채널별 RTSP 주소 조회 | O |
| `install.sh` | MediaMTX 바이너리 설치 | O |
| `start.sh` | 중계 기동 | O |
| `bin/` | MediaMTX 바이너리 | **X** (gitignore) |

카메라 주소와 계정은 `mediamtx.yml` 에 안 들어간다. `cameras.env` 의 값이
환경변수(`MTX_PATHS_CH1_SOURCE` …)로 주입되어 설정을 덮어쓴다. 그래서 설정 파일은
그대로 커밋해도 안전하다.

## 확인과 문제 해결

```bash
# 중계가 보는 경로 상태 (연결됨/대기 등)
curl -s http://127.0.0.1:9997/v3/paths/list | python3 -m json.tool

# 중계된 스트림을 직접 열어보기
ffplay -rtsp_transport tcp rtsp://<서버IP>:8554/ch1
```

| 증상 | 원인 | 조치 |
|---|---|---|
| `401 Unauthorized` (카메라 쪽) | `cameras.env` 의 계정·비번 | 비번에 `@ : / ? #` 있으면 URL 인코딩 (`@`→`%40`) |
| `404 Not Found` (카메라 쪽) | 채널 경로가 틀림 | `probe_onvif.py` 로 다시 받기. 계속 두드리면 계정 잠김 |
| `490 Account Blocked` | 이미 잠김 | 카메라 웹 UI 에서 잠금 해제 후 정확한 URL 로만 재시도 |
| Qt 에서만 끊김 | 무선 대역 포화 | `ip route get <카메라IP>` 로 wlan0 으로 나가는지 확인 → eth0 로 넘기기 |
| 어느 날 갑자기 연결 실패 | 카메라 IP 가 바뀜 | PNM 은 DHCP 라 IP 가 움직인다. 아래 "카메라 IP 고정" 참고 |
| `write queue is full` 로그 | 큐 부족 | `mediamtx.yml` 의 `writeQueueSize` 상향 (지연이 늘어남) |

## 카메라 IP 고정 (중계 쓰기 전에 반드시)

2026-08-03 현재 두 카메라의 실측 상태 (`ip neigh` 로 MAC 확인):

| IP | MAC | 카메라 | 할당 |
|---|---|---|---|
| 192.168.0.12 | `e4:30:22:eb:d7:f9` | PNO-A9081R (LAN 2) | **수동(고정)** |
| 192.168.0.13 | `e4:30:22:f2:d1:86` | PNM-C16083RVQ (LAN 3) | DHCP |

⚠️ **공유기 장치 목록은 PNM 을 192.168.0.12 로 표시하는데 이는 틀린 값이다.**
PNO 가 .12 를 고정으로 이미 쓰고 있어서, 공유기가 PNM 에게 .12 를 주려다 주소 충돌이
났고 PNM 이 .13 으로 밀려난 것이다. 공유기 표에는 처음 기록된 값이 그대로 남아 있다.
장비를 찾을 때는 표를 믿지 말고 MAC 으로 확인한다:

```bash
ping -c1 192.168.0.13 >/dev/null; ip neigh show 192.168.0.13
```

PNM 이 DHCP 인 채로 두면 IP 가 언제든 움직이고 `cameras.env` 가 조용히 깨진다.
중계를 상시로 쓸 거라면 **공유기에서 MAC `e4:30:22:f2:d1:86` 에 IP 예약**을 걸거나,
카메라 웹 UI 에서 DHCP 풀 **바깥** 주소로 고정하는 것이 먼저다 (PNO 의 .12 와
겹치지 않게).

## 안 볼 때는 안 당긴다

모든 경로가 `sourceOnDemand: true` 다. 보는 클라이언트가 없으면 카메라에서
아무것도 안 당겨오고, 마지막 시청자가 끊기고 10초 뒤 카메라 연결을 닫는다.
무선 대역과 카메라 세션을 아끼기 위한 설정이다.
