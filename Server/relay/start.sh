#!/bin/bash
# PNM-C16083RVQ 4채널 RTSP 패스스루 중계 기동.
#
#   ./start.sh            # 포그라운드 (로그를 눈으로 보며 확인할 때)
#   ./start.sh -d         # 백그라운드 + relay.log 로 기록
#
# 중계 결과는 아래 주소로 나온다 (<서버IP> = 이 파이의 주소):
#   rtsp://<서버IP>:8554/ch1 ... /ch4
set -euo pipefail

cd "$(dirname "$0")"

BIN=./bin/mediamtx
if [ ! -x "$BIN" ]; then
    echo "mediamtx 바이너리가 없습니다. 먼저 ./install.sh 를 실행하세요." >&2
    exit 1
fi

if [ ! -f cameras.env ]; then
    echo "cameras.env 가 없습니다:" >&2
    echo "  cp cameras.env.example cameras.env  후 카메라 주소·계정을 채우세요." >&2
    echo "  채널별 RTSP 경로는 probe_onvif.py 로 알아냅니다 (짐작 금지 — 계정 잠김)." >&2
    exit 1
fi

# cameras.env 의 값을 전부 환경변수로 내보낸다. MediaMTX 가 MTX_ 로 시작하는
# 변수를 읽어 mediamtx.yml 의 같은 이름 항목을 덮어쓴다 → 비밀이 yml 에 안 남는다.
set -a
# shellcheck disable=SC1091
source ./cameras.env
set +a

# 경로를 안 채운 채로 띄우면 카메라에 엉뚱한 URL 로 계속 두드리게 된다. 미리 막는다.
# 메인(CH1~4)은 필수, 서브(CH1S~4S)는 있으면 검사만 한다 — 2x2 미리보기를 안 쓸
# 수도 있으므로 없다고 막지는 않는다.
for ch in 1 2 3 4; do
    var="MTX_PATHS_CH${ch}_SOURCE"
    val="${!var:-}"
    if [ -z "$val" ]; then
        echo "$var 가 비어 있습니다 (cameras.env 확인)." >&2
        exit 1
    fi
    case "$val" in
        *PUT_CH*_PATH_HERE*)
            echo "$var 가 아직 템플릿 값입니다. probe_onvif.py 결과로 채우세요." >&2
            exit 1
            ;;
    esac
done

subs=0
for ch in 1 2 3 4; do
    var="MTX_PATHS_CH${ch}S_SOURCE"
    val="${!var:-}"
    [ -z "$val" ] && continue
    case "$val" in
        *PUT_CH*_PATH_HERE*)
            echo "$var 가 아직 템플릿 값입니다. 지우거나 실제 주소로 채우세요." >&2
            exit 1
            ;;
    esac
    subs=$((subs + 1))
done

if [ "${1:-}" = "-d" ]; then
    nohup "$BIN" ./mediamtx.yml >> relay.log 2>&1 &
    ip="$(ip route get 192.168.0.13 2>/dev/null | grep -oE 'src [0-9.]+' | awk '{print $2}')"
    [ -z "$ip" ] && ip="$(hostname -I | awk '{print $1}')"
    echo "중계 기동 (pid $!). 로그: $(pwd)/relay.log"
    echo "  메인 : rtsp://$ip:8554/ch1 ... /ch4    (2592x1520 30fps)"
    [ "$subs" -gt 0 ] && echo "  서브 : rtsp://$ip:8554/ch1s ... /ch4s  (저해상도, 2x2 미리보기용)"
else
    exec "$BIN" ./mediamtx.yml
fi
