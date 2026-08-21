if (DEFINED SDK_PATH)
    message("SDK_PATH: ${SDK_PATH}")
else ()
    set(SDK_PATH /opt/opensdk)
    message("SDK_PATH: default: ${SDK_PATH}")
endif ()

if(NOT DEFINED SOC)
  message(FATAL_ERROR "Variable SOC must be defined!")
endif()

message("SOC: ${SOC}")

if (SOC STREQUAL "cv5")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/cortex-a76-2022.08-gcc12.1-linux5.15/bin/aarch64-linux-gnu-")
elseif((SOC STREQUAL "nvidia_jp512") OR (SOC STREQUAL "orinnx8g_jp512"))
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/bootlin-toolchain-gcc-93/bin/aarch64-buildroot-linux-gnu-")
elseif(SOC STREQUAL "nt98538")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/aarch64-ca53-linux-gnueabihf-10.4.0/bin/aarch64-ca53-linux-gnu-")
elseif(SOC STREQUAL "wn9")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/aarch64-wn9-linux-gnu/bin/aarch64-wn9-linux-gnu-")
elseif(SOC STREQUAL "nt9869x")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/aarch64-ca73-linux-gnueabihf-10.4.0/bin/aarch64-ca73-linux-gnu-")
elseif(SOC STREQUAL "mt8139p")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/aarch64-poky-linux-mt8139p/sysroots/x86_64-oesdk-linux/usr/bin/aarch64-poky-linux/aarch64-poky-linux-")
elseif(SOC STREQUAL "nt98567")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/arm-ca7-linux-gnueabihf-10.4.0/bin/arm-ca7-linux-gnueabihf-")
elseif(SOC STREQUAL "wn8")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/aarch64-wn9-linux-gnu/bin/aarch64-wn9-linux-gnu-")
elseif(SOC STREQUAL "en683")
  set(TOOLCHAIN_PATH "${SDK_PATH}/toolchain/riscv/enx142/bin/riscv64-enx142-linux-gnu-")
endif()

add_compile_options(
  -Wall -Werror -Wformat -g
  -Werror=unused-result
  -Werror=sizeof-pointer-memaccess
  -Werror=array-bounds
  -Werror=nonnull
  -Wno-unused-function
  -Wno-extra
  -Wno-pedantic
  -Wno-unused-parameter
  -Wno-sign-compare
)

if(SOC STREQUAL "mt8139p")
  set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}clang" )
  set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}clang++" )
  set(CMAKE_SYSROOT "${SDK_PATH}/toolchain/aarch64-poky-linux-mt8139p/sysroots/aarch64-poky-linux")
  add_compile_options( 
    -Wno-braced-scalar-init
    -Wno-inconsistent-missing-override
    -Wno-return-stack-address
    -Wno-return-type
    -Wno-format-extra-args
  )
elseif(SOC STREQUAL "nt98567")
  set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}gcc" )
  set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}g++" )
  add_compile_options(
    -Werror=stringop-overflow
    -Werror=maybe-uninitialized
    -Werror=format-truncation
    -Werror=return-local-addr
    -Werror=stringop-truncation
    -Wno-psabi
    -Wno-format
  )
else()
  set(CMAKE_C_COMPILER   "${TOOLCHAIN_PATH}gcc" )
  set(CMAKE_CXX_COMPILER "${TOOLCHAIN_PATH}g++" )
  add_compile_options(
    -Werror=stringop-overflow
    -Werror=maybe-uninitialized
    -Werror=format-truncation
    -Werror=return-local-addr
    -Werror=stringop-truncation
  )
endif()
set(CMAKE_CXX_STANDARD 17 )
set(CMAKE_CXX_STANDARD_REQUIRED True )
