#!/bin/bash
# Start this repo instance, fully detached. Ports come from config.sh
# (copy config.sh.example to config.sh and adjust for your host).
# 스크립트가 있는 디렉토리 기준으로 실행 - 어디서 호출해도 리포의 web_gui.py를 띄운다
cd "$(dirname "$0")" || exit 1
[ -f config.sh ] && source config.sh
exec python3 -u web_gui.py "${TCP_PORT:-6000}" "${HTTP_PORT:-8081}" "${SNAPSHOT_PORT:-6001}" >> gui.log 2>&1
