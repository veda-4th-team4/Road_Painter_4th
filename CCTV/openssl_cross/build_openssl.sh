#!/usr/bin/env bash
#
# Cross-compile a STATIC OpenSSL for the Wisenet aarch64 (CV2x) camera target,
# using the OpenSDK linaro toolchain -- the same compiler the app is built with.
#
# Why this exists at all:
#   The SDK ships libssl/libcrypto for the target as RUNTIME shared objects
#   only -- no headers, no .a archives. So the TLS client (src/central_tls_sender.cpp)
#   cannot be compiled against them at all. This script produces the headers +
#   static archives the Makefile links.
#
# Why 1.1.1w and not the version the camera runs:
#   Read this before "fixing" the version -- the obvious answer is wrong.
#   The SDK's file is NAMED libssl.so.1.1.1b, but its CONTENTS report
#   "OpenSSL 1.1.1d  10 Sep 2019", and so does the camera's /usr/lib copy
#   (verified 2026-07-29: libssl md5 matches the SDK's byte for byte). The
#   original build here was 1.1.1b, chosen from that misleading FILENAME.
#
#   It does not actually matter which of the three we pick, because we link
#   STATICALLY: the app carries its own libcrypto/libssl and never loads the
#   camera's. There is no ABI to match. Given a free choice, 1.1.1w (Sep 2023,
#   the final 1.1.1 release) is simply the same API with the accumulated
#   security fixes, so nothing in the app source has to change.
#
#   Do NOT switch to 3.x without checking the link: it is a different ABI and
#   noticeably larger, and the app only needs a single outbound TLS client.
#
# Result: $PREFIX/include/openssl/*.h + $PREFIX/lib/lib{ssl,crypto}.a
#
# Usage (run inside the build container):
#   chmod +x openssl_cross/build_openssl.sh
#   ./openssl_cross/build_openssl.sh
#
# Install somewhere else (e.g. to compare against the current artifacts before
# replacing them):
#   PREFIX=/mnt/third_party/openssl-aarch64-test ./openssl_cross/build_openssl.sh
#
set -euo pipefail

OPENSSL_VER="${OPENSSL_VER:-1.1.1w}"
# Upstream SHA256 for openssl-1.1.1w.tar.gz. A tarball fetched over the network
# ends up statically linked into a binary that talks TLS -- verifying it is not
# ceremony. If you bump OPENSSL_VER you MUST update this too; the script fails
# closed rather than building whatever arrived.
OPENSSL_SHA256="${OPENSSL_SHA256:-cf3098950cb4d853ad95c0841f1f9c6d3dc102dccfcacd521d93925208b76ac8}"

# Default matches the Makefile's OPENSSL_ROOT, so a plain run drops the result
# exactly where the build expects it. (Note: opencv_cross/build_opencv.sh
# installs to /opt instead and has to be copied by hand -- deliberately not
# copied here.)
PREFIX="${PREFIX:-/mnt/third_party/openssl-aarch64}"
WORK="${WORK:-/opt/openssl-build}"
JOBS="$(nproc)"

TC_ROOT=/opt/opensdk/opensdk-5.00/linaro-aarch64-2020.09-gcc10.2-linux5.4
CROSS_PREFIX="$TC_ROOT/bin/aarch64-linux-gnu-"

if [ ! -x "${CROSS_PREFIX}gcc" ]; then
    echo "!! toolchain not found: ${CROSS_PREFIX}gcc" >&2
    echo "!! run this inside the OpenSDK build container." >&2
    exit 1
fi

echo ">> OpenSSL $OPENSSL_VER -> $PREFIX (static, aarch64/CV2x)"
mkdir -p "$WORK"
cd "$WORK"

TARBALL="openssl-$OPENSSL_VER.tar.gz"
if [ ! -f "$TARBALL" ]; then
    # openssl.org redirects old releases to github; -L follows it.
    curl -fL -o "$TARBALL.part" \
         "https://www.openssl.org/source/$TARBALL" \
      || curl -fL -o "$TARBALL.part" \
         "https://github.com/openssl/openssl/releases/download/OpenSSL_${OPENSSL_VER//./_}/$TARBALL"
    mv "$TARBALL.part" "$TARBALL"
fi

echo ">> verifying $TARBALL"
echo "$OPENSSL_SHA256  $TARBALL" | sha256sum -c -

rm -rf "openssl-$OPENSSL_VER"
tar xzf "$TARBALL"
cd "openssl-$OPENSSL_VER"

# no-shared  : we want .a only; the app links statically
# no-tests   : the test suite cannot run on the host anyway (cross build)
# no-dso     : no runtime engine loading -- dlopen in a static binary is a trap
# no-engine  : nothing here uses engines; drops a chunk of libcrypto
# no-comp    : TLS compression is a CRIME liability and unused
./Configure linux-aarch64 \
    --prefix="$PREFIX" \
    --openssldir="$PREFIX/ssl" \
    --cross-compile-prefix="$CROSS_PREFIX" \
    no-shared no-tests no-dso no-engine no-comp

make -j"$JOBS"

# install_sw skips man pages and the demo CA layout -- we only consume headers
# and archives, and the doc build is slow.
make install_sw

echo
echo ">> Done."
echo ">> headers: $PREFIX/include/openssl"
echo ">> archives:"
ls -la "$PREFIX/lib/libssl.a" "$PREFIX/lib/libcrypto.a"
echo ">> version string in the built headers:"
grep -o 'OPENSSL_VERSION_TEXT.*' "$PREFIX/include/openssl/opensslv.h" | head -1
echo
echo ">> Now build the app:  make"
