#!/usr/bin/env bash
# 빌드 -> 설치 -> 확인 한 방에. 이 세션에서 반복해온 5단계를 그대로 스크립트화한 것.
#
# 사용법:
#   ./build_install.sh              # 빌드 + 설치 + 상태 확인
#   ./build_install.sh --build-only # 빌드만 (설치 안 함)
#   ./build_install.sh --no-restore-https  # 디버깅 중 HTTPS 를 계속 열어두고 싶을 때
#
# 왜 이런 순서인가 (배경은 docs/archive/setup/OpenSDK 프로젝트 생성 및 설정 기록_v26.md 참고):
#   - docker compose up 이 아니라 직접 make 를 돌리는 이유: compose 안의 `make clean`
#     이 vboxsf 에서 자주 죽는다(§8). build_stale 로 우회한다.
#   - 설치 전후로 HTTPS 를 껐다 켜는 이유: 카메라가 HTTPSProprietary 전용이면
#     opensdk_install(HTTP 전용 도구)이 301 을 받고도 검사 없이 "Success" 를 찍는다(§7).
#     trap 으로 어떤 경로로 스크립트가 끝나도 HTTPS 는 반드시 원복한다.
#   - 설치 후 action=view 로 실제 InstalledApps/Status 를 확인하는 이유: 위와 같음 —
#     "Success" 출력을 믿지 않는다.
set -u

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
APP_NAME="$(basename "$APP_DIR")"
SOC="cv5"
SDK_IMAGE="opensdk:26.05.19_full"
CHANNEL="1"

# 카메라 접속 정보는 이 파일에 적지 않는다 — 이 스크립트는 커밋되고, 비밀번호는
# 한 번 커밋되면 되돌려도 히스토리에 남는다. camera.env(gitignore 대상)에서
# 읽거나 환경변수로 넘긴다:
#
#   cp camera.env.example camera.env && vi camera.env
#   CAMERA_PASS='...' ./build_install.sh        # 또는 이렇게 한 번만
#
# IP/USER 는 비밀이 아니라 설정이라 기본값을 둔다. 비밀번호만 기본값이 없다.
[ -f "$APP_DIR/camera.env" ] && . "$APP_DIR/camera.env"
CAMERA_IP="${CAMERA_IP:-192.168.0.13}"
CAMERA_USER="${CAMERA_USER:-admin}"
CAMERA_PASS="${CAMERA_PASS:-}"

BUILD_ONLY=0
RESTORE_HTTPS=1
for a in "$@"; do
  case "$a" in
    --build-only) BUILD_ONLY=1 ;;
    --no-restore-https) RESTORE_HTTPS=0 ;;
  esac
done

cd "$APP_DIR" || exit 1

echo "=== [1/4] 빌드 ($APP_NAME, SOC=$SOC) ==="
rm -rf app/build_stale 2>/dev/null
mv app/build app/build_stale 2>/dev/null
mkdir -p app/build
# cmake 와 make 를 한 docker run 안에서 &&로 이으면 vboxsf 동기화 지연 때문에
# "No rule to make target ... link.txt" 로 가끔 실패한다(cmake 가 쓴 파일을 바로
# 이어지는 make 가 못 보는 경우). 그래서 두 컨테이너 실행으로 나누고 sync 를 끼운다.
docker run --rm -v "$APP_DIR:/opt/$APP_NAME" -w "/opt/$APP_NAME" "$SDK_IMAGE" \
  bash -c "cd /opt/$APP_NAME/app/build && cmake -DSOC=$SOC .. >/dev/null && sync" \
  || { echo "cmake 실패"; exit 1; }
docker run --rm -v "$APP_DIR:/opt/$APP_NAME" -w "/opt/$APP_NAME" "$SDK_IMAGE" \
  bash -c "cd /opt/$APP_NAME/app/build && make && make install >/dev/null && \
           cd /opt/$APP_NAME && opensdk_packager -s $SOC" \
  || { echo "빌드 실패"; exit 1; }
rm -rf app/build_stale 2>/dev/null

CAP="$APP_DIR/$APP_NAME.cap"
[ -f "$CAP" ] || { echo "빌드는 끝났는데 $CAP 이 없음"; exit 1; }
echo "빌드 완료: $CAP ($(stat -c%s "$CAP") bytes, $(date -r "$CAP" '+%H:%M:%S'))"

[ "$BUILD_ONLY" = "1" ] && exit 0

# 설치 단계부터 자격증명이 필요하다. 빌드만 할 때는 없어도 되므로 여기서 검사한다.
# 빈 비밀번호로 진행하면 opensdk_install 이 인증 실패를 "Error" 한 줄로만 뱉고,
# 그 사이 SSL 정책은 이미 HTTP 로 내려가 있다 — 진단하기 나쁜 상태다.
[ -n "$CAMERA_PASS" ] || {
  echo "CAMERA_PASS 가 비어 있음. camera.env 를 만들거나 환경변수로 넘길 것:"
  echo "  cp camera.env.example camera.env && vi camera.env"
  exit 1
}

CURL=(curl -sk --digest -u "$CAMERA_USER:$CAMERA_PASS" --max-time 20)
API="https://$CAMERA_IP/stw-cgi"

restore_https() {
  [ "$RESTORE_HTTPS" = "1" ] || return 0
  curl -s --digest -u "$CAMERA_USER:$CAMERA_PASS" --max-time 20 \
    "http://$CAMERA_IP/stw-cgi/security.cgi?msubmenu=ssl&action=set&Policy=HTTPSProprietary" >/dev/null 2>&1
}
trap restore_https EXIT   # 빌드 이후 무슨 일이 있어도 HTTPS 는 되돌린다

echo "=== [2/4] HTTPS 임시 개방 (opensdk_install 은 HTTP 전용) ==="
"${CURL[@]}" "$API/security.cgi?msubmenu=ssl&action=set&Policy=HTTP" >/dev/null
sleep 5

echo "=== [3/4] 설치 ==="
[ -x ./opensdk_install ] || {
  cid=$(docker create "$SDK_IMAGE")
  docker cp "$cid:/opt/opensdk/common/bin/opensdk_install" ./opensdk_install
  docker rm "$cid" >/dev/null
  chmod +x ./opensdk_install
}
./opensdk_install -a "$APP_NAME" -i "$CAMERA_IP" -c "$CHANNEL" \
  -u "$CAMERA_USER" -w "$CAMERA_PASS" 2>&1 | grep -E "Upload Success|Install Success|Error"

sleep 2
restore_https
trap - EXIT
sleep 6

echo "=== [4/4] 확인 (Success 출력이 아니라 이 값을 믿을 것) ==="
"${CURL[@]}" "$API/opensdk.cgi?msubmenu=apps&action=view" | grep -E "InstalledApps|AppName=$APP_NAME|Status|InstalledDate" | head -8
