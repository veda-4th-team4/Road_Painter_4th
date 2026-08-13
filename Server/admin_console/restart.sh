#!/usr/bin/env bash
# admin_console(8083) 인스턴스 재시작. cctv_calibration_manager/restart.sh와
# 동일한 이유로 스크립트 파일 안에서 pkill한다 (ssh로 직접 pkill -f를 보내면
# 그 문자열이 원격 셸 자신의 커맨드라인에도 잡혀서 자기 자신을 죽인다).
set -u
DIR="$HOME/Road_Painter_4th/Server/admin_console"
cd "$DIR" || exit 1
set -a
[ -f config.sh ] && source config.sh
set +a

PIDS=$(pgrep -f "web_gui\.py ${TCP_PORT:-6100} ${HTTP_PORT:-8083} ${SNAPSHOT_PORT:-6101}" || true)
if [ -n "$PIDS" ]; then
  echo "정지: $PIDS"
  kill $PIDS
  for _ in 1 2 3 4 5; do
    sleep 1
    pgrep -f "web_gui\.py ${TCP_PORT:-6100} ${HTTP_PORT:-8083} ${SNAPSHOT_PORT:-6101}" >/dev/null || break
  done
fi

setsid nohup python3 -u web_gui.py "${TCP_PORT:-6100}" "${HTTP_PORT:-8083}" "${SNAPSHOT_PORT:-6101}" >> web_gui.log 2>> gui_err.log < /dev/null &
sleep 4

CODE=$(curl -s -o /dev/null -w '%{http_code}' -m 5 http://127.0.0.1:${HTTP_PORT:-8083}/ || echo 000)
echo "기동: $(pgrep -f "web_gui\.py ${TCP_PORT:-6100}" | tr '\n' ' ')"
echo "HTTP: $CODE"
[ "$CODE" = "200" ]
