#!/bin/bash
# Start this repo instance, fully detached. Ports come from config.sh
# (copy config.sh.example to config.sh and adjust for your host).
# 스크립트가 있는 디렉토리 기준으로 실행 - 어디서 호출해도 리포의 web_gui.py를 띄운다
cd "$(dirname "$0")" || exit 1
# set -a: config.sh의 변수를 전부 export해서 web_gui.py가 os.environ으로 읽는
# RP_CCTV_BRIDGE/RP_SERVER_HOST/RP_SERVER_PORT 같은 값도 전달되게 한다
# (source만 하면 셸 변수로만 남고 자식 프로세스인 python엔 안 넘어감).
set -a
[ -f config.sh ] && source config.sh
set +a
exec python3 -u web_gui.py "${TCP_PORT:-6000}" "${HTTP_PORT:-8081}" "${SNAPSHOT_PORT:-6001}" >> gui.log 2>&1
