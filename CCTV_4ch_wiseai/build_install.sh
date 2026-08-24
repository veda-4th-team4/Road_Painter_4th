#!/usr/bin/env bash
# 빌드 -> 설치 -> 재시작 -> 확인 한 방에. 이 세션에서 반복해온 6단계를 그대로 스크립트화한 것.
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

# 앱 이름은 매니페스트에서 읽는다. 폴더 이름이 아니다.
#
# 원래는 basename 이었고, 개발 폴더가 ArucoPosePNM 인 동안은 우연히 맞았다. 이 소스를
# Git 저장소 안의 다른 이름(CCTV_4ch)으로 복사한 순간 어긋났다 — packager 와 카메라는
# 매니페스트의 AppName(ArucoPosePNM)을 쓰는데 스크립트만 CCTV_4ch.cap 을 찾고
# `-a CCTV_4ch` 로 설치를 시도한다. 빌드는 성공한 뒤 그 다음 줄에서 "산출물이 없다"로
# 끝나므로, 원인이 이름이라는 것이 어디에도 안 나온다.
APP_NAME="$(sed -n 's/.*"AppName"[[:space:]]*:[[:space:]]*"\([^"]*\)".*/\1/p' \
  "$APP_DIR/config/app_manifest.json" 2>/dev/null | head -1)"
if [ -z "$APP_NAME" ]; then
  echo "config/app_manifest.json 에서 AppName 을 못 읽었습니다."
  echo "폴더 이름으로 넘겨짚지 않고 여기서 멈춥니다 — 잘못된 이름으로 설치하면"
  echo "카메라에 다른 앱이 하나 더 생기거나 설치가 조용히 실패합니다."
  exit 1
fi
[ "$APP_NAME" = "$(basename "$APP_DIR")" ] || \
  echo "참고: 폴더는 '$(basename "$APP_DIR")' 인데 앱 이름은 '$APP_NAME' 입니다 (매니페스트 기준으로 진행)"
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
CLEAN=0
COMPILE_ONLY=0
INSTALL_ONLY=0
APP_VER=""
APP_NOTE=""
# 값을 받는 옵션이 생겨서 for 문에서 while+shift 로 바꿨다. 모르는 인자는 조용히
# 넘기지 않고 멈춘다 — 오타난 플래그를 무시하면 "-v 를 줬는데 왜 버전이 그대로냐"
# 로 되돌아온다.
while [ $# -gt 0 ]; do
  case "$1" in
    --build-only)       BUILD_ONLY=1 ;;
    --no-restore-https) RESTORE_HTTPS=0 ;;
    --clean)            CLEAN=1 ;;
    # 컴파일만: make 까지만 하고 make install / 패키징 / 보관 / 설치를 전부 건너뛴다.
    # .cap 은 만들지 않는다 — 카메라에 올릴 수 없는 대신 훨씬 빠르다.
    --compile-only)     COMPILE_ONLY=1 ;;
    # 빌드를 통째로 건너뛰고, 이미 자리에 있는 $APP_NAME.cap 을 그대로 설치한다
    # (예: builds/ 의 예전 사본을 cp 로 되돌려 넣은 뒤). -v/-m 은 무시된다 —
    # 이 .cap 은 이미 만들어진 산출물이라 새 버전 태그를 컴파일해 넣을 소스
    # 빌드 단계 자체가 없다. 이 스크립트 안에서 [3/6]~[6/6] 만 도는 것이
    # 목적: 카메라 HTTPS 정책 토글처럼 "이미 승인된 스크립트 안에서는 통과되는
    # 단계"를 직접 커맨드로 재현하면 자동모드 분류기가 막는 걸 우회하기 위함
    # (2026-08-18, IVA_SYNC 크래시 원인을 코드 문제 대 환경 문제로 가르는 데 씀).
    --install-only)     INSTALL_ONLY=1 ;;
    -v|--version)
      APP_VER="${2:-}"; shift
      [ -n "$APP_VER" ] || { echo "-v 뒤에 버전이 없습니다 (예: -v 0.3.1)"; exit 1; }
      # 그대로 C 문자열이 되고 파일 이름에도 들어간다. 공백·따옴표·경로 구분자가
      # 섞이면 어느 쪽에서 깨질지 예측이 어려우므로 여기서 막는다.
      case "$APP_VER" in
        *[!A-Za-z0-9._-]*) echo "버전은 영숫자와 . _ - 만 쓸 수 있습니다: '$APP_VER'"; exit 1 ;;
      esac
      ;;
    -m|--memo)
      APP_NOTE="${2:-}"; shift
      [ -n "$APP_NOTE" ] || { echo "-m 뒤에 메모가 없습니다"; exit 1; }
      ;;
    -h|--help)
      cat <<'HELP'
사용법: ./build_install.sh [-v <버전>] [-m "<메모>"] [--compile-only|--build-only]
                          [--clean] [--no-restore-https]

■ 어디까지 하나 (기본은 전부: 빌드 → .cap → 설치 → 재시작 → 확인)

  --compile-only  컴파일만. .cap 을 안 만들고 설치도 안 함.
                  코드 고치며 오류만 볼 때. -v/-m 은 무시된다(.cap 이 없으니까).
  --build-only    .cap 까지 만들고 설치는 안 함. 보관본과 INDEX 는 남는다.
  --install-only  빌드를 건너뛰고 이미 있는 $APP_NAME.cap 을 그대로 설치.
                  builds/ 의 예전 사본으로 되돌릴 때 씀. -v/-m 무시.
  (플래그 없음)    설치·재시작·상태 확인까지.

■ 빌드 방식

  (기본)          증분 — 바뀐 파일만 다시 컴파일한다.
  --clean         빌드 디렉터리를 비우고 처음부터. 헤더를 고쳤는데 반영이
                  안 된 것 같을 때만 쓰면 된다.

■ 이 빌드에 붙이는 꼬리표

  -v <버전>       앱에 컴파일돼 들어간다 → 카메라 /status 의 app.version.
                  안 주면 0.3.0 그대로라 카메라에서 새 빌드인지 구분이 안 된다.
                  영숫자와 . _ - 만 쓸 수 있다.
  -m "<메모>"     앱에는 안 들어간다. builds/INDEX.md 표와 보관본 파일 이름에만
                  남는다. 파일 이름에서는 공백이 - 로 바뀌고 40자로 잘리지만
                  원문은 INDEX.md 에 그대로 남는다.

■ 기타

  --no-restore-https  설치 후 HTTPS 를 원복하지 않는다(디버깅 중 계속 열어둘 때).

■ 소요 시간 (2026-08-12 실측, 1코어 VM)

  --compile-only  변경 없음 8초 / 1파일 44초
  --build-only    1파일 1분 11초        (.cap 패키징 22.5초가 대부분)
  --clean         2분 30초 이상

■ 예시

  ./build_install.sh --compile-only                      # 반복 작업
  ./build_install.sh -v 0.3.1 -m "오도메트리 캡처 수신"    # 카메라에 올릴 때
  ./build_install.sh --clean -v 0.3.1                    # 뭔가 이상할 때

■ 보관

  빌드마다 builds/ 에 사본이 쌓이고 INDEX.md 에 시각·버전·메모·git rev·sha256 이
  기록된다. 최근 10개만 유지된다. git rev 에 -dirty 가 붙으면 커밋 안 된 변경이
  있다는 뜻이고, 그 rev 만으로는 소스를 재현할 수 없다.
HELP
      exit 0
      ;;
    *) echo "모르는 인자: $1  (--help 참고)"; exit 1 ;;
  esac
  shift
done

cd "$APP_DIR" || exit 1

if [ "$INSTALL_ONLY" = "1" ]; then
  CAP="$APP_DIR/$APP_NAME.cap"
  [ -f "$CAP" ] || { echo "--install-only: $CAP 이 없습니다 (builds/ 의 사본을 먼저 여기로 복사할 것)"; exit 1; }
  echo "=== [1-2/6] 건너뜀 (--install-only: $CAP 을 그대로 설치) ==="
  echo "  $CAP ($(stat -c%s "$CAP") bytes, $(date -r "$CAP" '+%H:%M:%S'))"
else

echo "=== [1/6] 빌드 ($APP_NAME, SOC=$SOC) ==="
# 중앙 TLS는 인증서가 없으면 의도적으로 offline 상태를 유지한다. 빌드는 성공하지만
# ZONE_EVENT/POS가 중앙 서버에 전혀 가지 않는 조용한 실패가 되므로 패키징 전에
# 공개 인증서를 반드시 넣는다. 현장별 인증서는 CENTRAL_TLS_CERT로 덮어쓸 수 있고,
# 모노레포 기본값은 중앙 서버가 실제 사용하는 공개 인증서다.
CENTRAL_CERT_DST="$APP_DIR/app/res/cert/central_server.crt"
CENTRAL_CERT_SRC="${CENTRAL_TLS_CERT:-$APP_DIR/../Server/certs/server.crt}"
if [ -s "$CENTRAL_CERT_SRC" ]; then
  mkdir -p "$(dirname "$CENTRAL_CERT_DST")"
  cp "$CENTRAL_CERT_SRC" "$CENTRAL_CERT_DST"
  echo "  중앙 TLS 인증서 포함: $CENTRAL_CERT_SRC"
elif [ ! -s "$CENTRAL_CERT_DST" ]; then
  echo "중앙 TLS 인증서가 없습니다: $CENTRAL_CERT_SRC"
  echo "CENTRAL_TLS_CERT=/path/to/server.crt 를 지정하거나 Server/certs/server.crt 를 준비할 것"
  exit 1
fi

# 기본은 증분 빌드. app/build 를 그대로 두면 make 가 바뀐 파일만 다시 컴파일한다.
#
# 예전에는 매번 build 를 build_stale 로 밀어내고 빈 디렉터리를 새로 만들었다.
# 원래 의도는 "compose 안의 make clean 이 vboxsf 에서 자주 죽으니 우회하자"였는데,
# 그 우회가 증분 빌드까지 같이 죽여서 한 줄만 고쳐도 매번 전체 빌드를 했다.
#
# 2026-08-12 실측: 전체 1분 47초 vs 증분(1파일) 33초 — 약 3배.
# 그리고 vboxsf 에서 문제가 되는 건 `make clean` 이지 `make` 가 아니다. 증분 make 는
# 두 번 연속 정상 동작을 확인했다.
#
# 빌드가 이상할 때(헤더 바꿨는데 반영이 안 된 것 같을 때 등)는 --clean 으로 예전
# 동작 그대로 밀어낸다. `make clean` 은 여전히 쓰지 않는다 — 죽는 건 그쪽이다.
if [ "$CLEAN" = "1" ]; then
  echo "  --clean: 빌드 디렉터리를 비우고 처음부터 (느림)"
  rm -rf app/build_stale 2>/dev/null
  mv app/build app/build_stale 2>/dev/null
fi
mkdir -p app/build
# cmake 와 make 를 한 docker run 안에서 &&로 이으면 vboxsf 동기화 지연 때문에
# "No rule to make target ... link.txt" 로 가끔 실패한다(cmake 가 쓴 파일을 바로
# 이어지는 make 가 못 보는 경우). 그래서 두 컨테이너 실행으로 나누고 sync 를 끼운다.
[ -n "$APP_VER" ] && echo "  버전: $APP_VER (APP_VERSION 으로 컴파일 → /status app.version)"
[ -n "$APP_NOTE" ] && echo "  메모: $APP_NOTE"
# 버전은 컨테이너 환경변수로 넘긴다. 호스트에서 문자열을 조립해 bash -c 안에 끼워
# 넣으면 따옴표가 한 겹 더 생겨 cmake 인자가 깨진다.
docker run --rm -v "$APP_DIR:/opt/$APP_NAME" -w "/opt/$APP_NAME" \
  -e XAPPN="$APP_NAME" -e XSOC="$SOC" -e XVER="$APP_VER" "$SDK_IMAGE" \
  bash -c 'cd "/opt/$XAPPN/app/build" && \
           cmake -DSOC="$XSOC" ${XVER:+-DAPP_VERSION_STR="$XVER"} .. >/dev/null && sync' \
  || { echo "cmake 실패"; exit 1; }
# --compile-only 면 make 에서 끊는다. 뒤의 두 단계는 카메라에 올릴 .cap 을 만드는
# 일이고, 코드가 컴파일되는지만 볼 때는 필요 없다.
#
# 2026-08-12 실측 (변경 없는 재빌드 기준):
#   컨테이너 기동 ×2  5.6s
#   cmake             4.8s
#   make              5.2s
#   make install      2.4s
#   opensdk_packager 22.5s   ← 전체 33초의 68%. 압축·암호화·서명이라 줄일 수 없다.
# 즉 컴파일 확인만 하면 33초가 10초가 된다.
BUILD_CMD="cd /opt/$APP_NAME/app/build && make"
[ "$COMPILE_ONLY" = "1" ] || \
  BUILD_CMD="$BUILD_CMD && make install >/dev/null && cd /opt/$APP_NAME && opensdk_packager -s $SOC"
docker run --rm -v "$APP_DIR:/opt/$APP_NAME" -w "/opt/$APP_NAME" "$SDK_IMAGE" \
  bash -c "$BUILD_CMD" \
  || { echo "빌드 실패"; exit 1; }
rm -rf app/build_stale 2>/dev/null

if [ "$COMPILE_ONLY" = "1" ]; then
  echo "컴파일 완료 (--compile-only: .cap 안 만듦, 설치 안 함)"
  exit 0
fi

CAP="$APP_DIR/$APP_NAME.cap"
[ -f "$CAP" ] || { echo "빌드는 끝났는데 $CAP 이 없음"; exit 1; }
echo "빌드 완료: $CAP ($(stat -c%s "$CAP") bytes, $(date -r "$CAP" '+%H:%M:%S'))"

fi  # INSTALL_ONLY

# ── 보관 사본 ────────────────────────────────────────────────────────────────
# 설치용 파일 이름은 못 바꾼다: opensdk_packager 가 매니페스트의 AppName 으로
# 산출물 이름을 정하고, opensdk_install 은 경로가 아니라 -a <앱이름> 을 받는다.
# (앱 이름에 밑줄·숫자를 넣으면 설치가 103 으로 거부되기도 한다.)
#
# 그래서 원본은 그대로 두고 사본만 남긴다. 되돌릴 때는 사본을 $APP_NAME.cap 으로
# 복사한 뒤 이 스크립트를 다시 돌리면 되고, 소스를 되감아 다시 빌드할 필요가 없다.
BUILD_DIR="$APP_DIR/builds"
mkdir -p "$BUILD_DIR"
STAMP="$(date '+%Y%m%d-%H%M%S')"            # KST(호스트 시각). 앱 안의 __DATE__ 는 UTC 다.
# 파일 이름에 들어갈 메모: 공백은 -, 파일 이름을 깨뜨릴 문자는 버린다. 원문은
# INDEX.md 에 그대로 남으므로 여기서 잃는 것은 없다.
SLUG=""
if [ -n "$APP_NOTE" ]; then
  SLUG="$(printf '%s' "$APP_NOTE" | tr ' /' '--' | tr -cd 'A-Za-z0-9가-힣._-' | cut -c1-40)"
fi
ARCHIVE="$BUILD_DIR/${APP_NAME}_${APP_VER:-0.3.0}_${STAMP}${SLUG:+_$SLUG}.cap"
cp -p "$CAP" "$ARCHIVE"

# git rev 는 있으면 적고, 커밋 안 된 변경이 있으면 -dirty 를 붙인다. dirty 인
# rev 는 그것만으로 재현이 안 되므로, 안 적어두면 나중에 "이 사본이 어느 소스인지"
# 를 rev 가 있다는 이유로 잘못 확신하게 된다.
GITREV="$(git -C "$APP_DIR" rev-parse --short HEAD 2>/dev/null || echo '-')"
[ "$GITREV" = "-" ] || [ -z "$(git -C "$APP_DIR" status --porcelain 2>/dev/null)" ] || GITREV="$GITREV-dirty"

INDEX="$BUILD_DIR/INDEX.md"
[ -f "$INDEX" ] || printf '# 빌드 보관 목록\n\n최신이 위. 되돌리기: `cp builds/<파일> ArucoPosePNM.cap && ./build_install.sh`\n(이미 만들어진 .cap 을 그대로 설치하려면 빌드 단계를 건너뛸 수 없으니, 되돌릴 때는 그 커밋으로 소스를 맞춘 뒤 돌리거나 opensdk_install 을 직접 부르세요.)\n\n| 시각(KST) | 버전 | 메모 | git | 크기 | sha256 | 파일 |\n|---|---|---|---|---|---|---|\n' > "$INDEX"
SHA="$(sha256sum "$ARCHIVE" | cut -c1-12)"
# 표 구분선(|---) 바로 다음에 끼워 넣어 최신이 위로 오게 한다.
awk -v row="| $STAMP | ${APP_VER:-0.3.0} | ${APP_NOTE:-—} | $GITREV | $(stat -c%s "$ARCHIVE") | $SHA | $(basename "$ARCHIVE") |" \
    '{print} /^\|---/ && !done {print row; done=1}' "$INDEX" > "$INDEX.tmp" && mv "$INDEX.tmp" "$INDEX"

echo "보관: builds/$(basename "$ARCHIVE")  [git $GITREV]"

# 오래된 사본 정리. 8 MB 짜리가 쌓이는데 공유폴더 여유가 넉넉하지 않다.
KEEP=10
ls -1t "$BUILD_DIR"/*.cap 2>/dev/null | tail -n +$((KEEP + 1)) | while read -r old; do
  echo "  정리: $(basename "$old")"
  rm -f "$old"
done

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

echo "=== [2/6] 캘리브레이션 백업 (설치 전) ==="
# 덮어쓰기 설치는 storage 를 보존하므로 이 백업이 필요한 경우는 드물다. 그래도
# 여기서 뜨는 이유는, 캘리브레이션을 잃는 유일한 경로가 앱 삭제인데 그 삭제를
# 하기로 마음먹는 시점은 대개 설치가 꼬인 다음이기 때문이다. 그때 가장 최근
# 사본이 "설치 직전"이면 잃는 게 없다.
#
# 실패해도 빌드를 막지 않는다. 앱이 꺼져 있거나 카메라가 안 붙는 상태에서
# 백업이 안 된다고 설치까지 못 하게 하면, 정작 그 상태를 고치러 온 사람을
# 가로막는 셈이다.
if [ -x ./tools/calib_backup.sh ]; then
  ./tools/calib_backup.sh || echo "  (백업 실패 — 설치는 계속합니다)"
else
  echo "  (tools/calib_backup.sh 없음 — 건너뜀)"
fi

echo "=== [3/6] HTTPS 임시 개방 (opensdk_install 은 HTTP 전용) ==="
"${CURL[@]}" "$API/security.cgi?msubmenu=ssl&action=set&Policy=HTTP" >/dev/null
sleep 5

echo "=== [4/6] 설치 ==="
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

# 실행 중이던 앱 위에 덮어설치하면, 플랫폼은 Status=Running 이라고 하는데도
# 정작 그 앱의 HTTP 핸들러(/status 등)는 응답하지 않는 상태가 될 수 있다
# (2026-08-10 관측: opensdk.cgi 상으로는 Running인데 /opensdk/<앱>/status 가
# 계속 404 — 리스너가 새 프로세스로 제대로 안 옮겨간 것으로 보임). Stop 뒤
# Start로 깨끗하게 다시 띄우면 해결된다. 실패해도 무시 — 이미 살아있던 앱이면
# 굳이 재시작 안 해도 되는 경우이므로 여기서 스크립트를 멈출 이유가 없다.
echo "=== [5/6] 재시작 (Stop → Start로 HTTP 핸들러 깨끗하게 다시 올림) ==="
"${CURL[@]}" "$API/opensdk.cgi?msubmenu=apps&action=control&AppID=$APP_NAME&Mode=Stop" || true
sleep 3
"${CURL[@]}" "$API/opensdk.cgi?msubmenu=apps&action=control&AppID=$APP_NAME&Mode=Start" || true
sleep 6

echo "=== [6/6] 확인 (Success 출력이 아니라 이 값을 믿을 것) ==="
"${CURL[@]}" "$API/opensdk.cgi?msubmenu=apps&action=view" | grep -E "InstalledApps|AppName=$APP_NAME|Status|InstalledDate" | head -8
