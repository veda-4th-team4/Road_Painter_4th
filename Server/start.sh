#!/bin/bash
# Road-Painter 서버 + 관리자 창(웹 GUI) 통합 실행.
# 서버를 켜면 관리자 창(admin_console)이 자동으로 함께 떠서, Qt가 언제든
# http://<서버IP>:<HTTP_PORT> (현 서버 8083 - admin_console/config.sh)를 열어
# 캘리브레이션/점검을 할 수 있게 한다.
# (web_gui.py는 서버가 죽어 있어도 3초마다 재접속하므로 시작 순서 무관 —
#  서버를 껐다 켜도 웹 창은 계속 떠 있고, 다음 실행 때 아래에서 재사용된다)
cd "$(dirname "$0")" || exit 1

# 관리자 창 HTTP 포트 (admin_console/config.sh에서 바꿨으면 따라감)
HTTP_PORT=$(source admin_console/config.sh 2>/dev/null; echo "${HTTP_PORT:-8081}")

# 이미 그 포트에서 서비스 중이면 재사용, 아니면 백그라운드로 자동 실행.
# setsid: 서버를 Ctrl+C로 꺼도 웹 창은 살아남게 세션 분리.
if ss -tln 2>/dev/null | grep -q ":${HTTP_PORT} "; then
    echo "[start] 관리자 창 이미 실행 중 - http://<서버IP>:${HTTP_PORT} 재사용"
else
    setsid ./admin_console/start.sh < /dev/null &
    echo "[start] 관리자 창(웹 GUI) 시작 - http://<서버IP>:${HTTP_PORT} (로그: admin_console/gui.log)"
fi

exec ./server
