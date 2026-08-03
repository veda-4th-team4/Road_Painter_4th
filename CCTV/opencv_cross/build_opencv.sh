#!/usr/bin/env bash
#
# Cross-compile a minimal, STATIC OpenCV (with the contrib ArUco module) for the
# Wisenet aarch64 camera target, using the OpenSDK linaro toolchain.
#
# Result: static libs + headers installed to $PREFIX, ready to be statically
# linked into the app. Only the modules the app needs are built:
#   aruco -> pulls in calib3d, features2d, flann, imgproc, core.
# imgcodecs (JPEG/PNG) is intentionally NOT built to keep the binary small; it
# is only needed if you enable SAVE_DEBUG_JPG in aruco_detector_cv.cpp.
#
# Usage (run inside the build container):
#   chmod +x opencv_cross/build_opencv.sh
#   ./opencv_cross/build_opencv.sh
#
set -euo pipefail

OPENCV_VER=4.6.0
PREFIX=/opt/opencv-aarch64
WORK=/opt/opencv-build
JOBS="$(nproc)"

# Absolute path to the toolchain file (this script's directory).
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLCHAIN="$SCRIPT_DIR/aarch64-toolchain.cmake"

echo ">> OpenCV $OPENCV_VER -> $PREFIX (static, aarch64)"
mkdir -p "$WORK"
cd "$WORK"

if [ ! -d opencv ]; then
    git clone --depth 1 -b "$OPENCV_VER" https://github.com/opencv/opencv.git
fi
if [ ! -d opencv_contrib ]; then
    git clone --depth 1 -b "$OPENCV_VER" https://github.com/opencv/opencv_contrib.git
fi

rm -rf opencv/build
mkdir -p opencv/build
cd opencv/build

cmake .. \
    -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$PREFIX" \
    -DOPENCV_EXTRA_MODULES_PATH="$WORK/opencv_contrib/modules" \
    -DBUILD_LIST=aruco \
    -DBUILD_SHARED_LIBS=OFF \
    -DOPENCV_GENERATE_PKGCONFIG=ON \
    -DBUILD_ZLIB=ON \
    -DBUILD_opencv_apps=OFF -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_DOCS=OFF \
    -DBUILD_JAVA=OFF -DBUILD_opencv_python2=OFF -DBUILD_opencv_python3=OFF \
    -DWITH_1394=OFF -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF \
    -DWITH_GTK=OFF -DWITH_QT=OFF -DWITH_V4L=OFF \
    -DWITH_JPEG=OFF -DWITH_PNG=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF -DWITH_OPENEXR=OFF \
    -DWITH_OPENCL=OFF -DWITH_CUDA=OFF -DWITH_IPP=OFF -DWITH_ITT=OFF \
    -DWITH_PROTOBUF=OFF -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF -DWITH_TBB=OFF -DWITH_PTHREADS_PF=ON

make -j"$JOBS"
make install

echo
echo ">> Done. Installed to $PREFIX"
echo ">> pkg-config file: $PREFIX/lib/pkgconfig/opencv4.pc"
echo ">> Now build the app:  make"
