# CMake toolchain file: cross-compile for the Wisenet aarch64 target using the
# OpenSDK 5.00 linaro toolchain (same compiler the app is built with).

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(TC_ROOT /opt/opensdk/opensdk-5.00/linaro-aarch64-2020.09-gcc10.2-linux5.4)
set(TC_SYSROOT ${TC_ROOT}/aarch64-linux-gnu/libc)

set(CMAKE_C_COMPILER   ${TC_ROOT}/bin/aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER ${TC_ROOT}/bin/aarch64-linux-gnu-g++)

set(CMAKE_SYSROOT        ${TC_SYSROOT})
set(CMAKE_FIND_ROOT_PATH ${TC_SYSROOT})

# Search host for programs, target sysroot for libs/headers.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# aarch64 implies NEON/VFP; keep it generic for the camera SoC.
set(CMAKE_C_FLAGS_INIT   "-O2")
set(CMAKE_CXX_FLAGS_INIT "-O2")
