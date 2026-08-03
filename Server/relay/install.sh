#!/bin/bash
# MediaMTX 바이너리를 bin/ 으로 받아둔다 (리포에는 커밋하지 않는다 — 60MB).
#
#   ./install.sh            # 기본 버전 설치
#   MTX_VERSION=v1.19.3 ./install.sh
set -euo pipefail

cd "$(dirname "$0")"

VERSION="${MTX_VERSION:-v1.19.3}"

case "$(uname -m)" in
    aarch64|arm64) ARCH=linux_arm64 ;;
    armv7l)        ARCH=linux_armv7 ;;
    x86_64)        ARCH=linux_amd64 ;;
    *) echo "지원하지 않는 아키텍처: $(uname -m)" >&2; exit 1 ;;
esac

URL="https://github.com/bluenviron/mediamtx/releases/download/${VERSION}/mediamtx_${VERSION}_${ARCH}.tar.gz"

echo "다운로드: $URL"
mkdir -p bin
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT

curl -fsSL -o "$tmp/mtx.tgz" "$URL"
tar xzf "$tmp/mtx.tgz" -C "$tmp"
install -m 755 "$tmp/mediamtx" bin/mediamtx
# 번들 기본 설정은 참고용으로만 남긴다 (우리는 mediamtx.yml 을 쓴다).
[ -f "$tmp/mediamtx.yml" ] && cp "$tmp/mediamtx.yml" bin/mediamtx.default.yml

echo "설치 완료: $(./bin/mediamtx --version)"
echo
echo "다음 순서:"
echo "  1) cp cameras.env.example cameras.env"
echo "  2) python3 probe_onvif.py --host <카메라IP> --user admin --password '<비번>' --env"
echo "  3) 출력된 MTX_PATHS_CH*_SOURCE 를 cameras.env 에 붙여넣기"
echo "  4) ./start.sh"
