set(TARGET_LIB sample_component)
set(THIRD_PARTY ${CMAKE_HOME_DIRECTORY}/third_party)

add_library(${TARGET_LIB} MODULE
    sample_component.cc
    aruco_processor.cc
    pose_sender.cc
    central_tls_sender.cc
    central_cmd_parse.cc
    intrinsics_calib.cc
    homography_mapper.cc
    dyn_roi.cc
    proximity_guard.cc
    wiseai_metadata.cc)

# 빌드마다 어느 빌드인지 구분되게 하는 값. build_install.sh 의 -v 가 넘긴다.
#
# 셸에서 -DCMAKE_CXX_FLAGS='-DAPP_VERSION=\"0.3.1\"' 로 넣는 방법도 있지만, 그
# 경로는 셸 -> cmake -> make -> sh 를 거치며 따옴표가 한 겹씩 벗겨져서 결국
# APP_VERSION 이 문자열이 아니라 0.3.1 이라는 식으로 전개돼 컴파일이 깨진다.
# target_compile_definitions 는 cmake 가 인용을 책임지므로 그 문제가 없다.
#
# 안 주면 정의 자체가 없고, app_config.h 의 #ifndef 기본값(0.3.0)이 그대로 쓰인다.
if(APP_VERSION_STR)
    target_compile_definitions(${TARGET_LIB} PRIVATE APP_VERSION="${APP_VERSION_STR}")
endif()

target_include_directories(${TARGET_LIB} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${THIRD_PARTY}/opencv-aarch64/include/opencv4
    # p_raw_image.h pulls in raw_image.h, which lives under the NPU API tree.
    ${WISENET_PRODUCT_RELEASE_PATH}/lib/npu_api/includes
    # openssl/ssl.h for central_tls_sender.cc. Not under WISENET_PRODUCT_RELEASE_PATH
    # (that only has the runtime .so, no headers) — verified 2026-08-10 by cross-
    # compiling a link probe with this exact include/lib pair against the cv5
    # toolchain; see docs/ (OpenSSL link verification).
    ${COMMON}/opensource/${SOC}/openssl/include)

target_link_directories(${TARGET_LIB} PRIVATE
    ${THIRD_PARTY}/opencv-aarch64/lib
    # OpenCV's bundled 3rd-party archives (carotene/NEON HAL, zlib, openjp2).
    ${THIRD_PARTY}/opencv-aarch64/lib/opencv4/3rdparty)

# NOTE: the 3rdparty archives MUST come after the opencv_* ones. libopencv_core
# and libopencv_imgproc reference carotene_o4t::* from libtegra_hal; a MODULE
# library tolerates unresolved symbols at link time, so omitting these still
# builds — and then dlopen() fails on the camera and the app dies at start-up
# with no message beyond "StartedNotRunning".
target_link_libraries(${TARGET_LIB} PRIVATE
    # IPLVideoFrameRaw (instantiated in ProcessRawVideo) lives here. Without it
    # the vtable is missing and dlopen() fails on the camera.
    ${SOC}_technical_streamingService_sharedBuffer
    # NOTE: do NOT link `component` here. Event::GetBlobArgument() and
    # BaseObject::operator new resolve at runtime from the host executable
    # (app/bin/<AppName>), which already links libcomponent.a — the untouched
    # template leaves them unresolved too and runs fine. Linking it into the
    # module breaks the build.
    opencv_aruco opencv_calib3d opencv_features2d opencv_flann
    opencv_imgproc opencv_core          # 이 순서를 바꾸지 말 것
    tegra_hal zlib libopenjp2
    # ssl/crypto resolve from WISENET_PRODUCT_RELEASE_PATH/3rd_party/lib, already
    # on the link path via app/CMakeLists.txt's global link_directories() — no
    # new link_directories() needed here. Runtime: libssl.so.3/libcrypto.so.3
    # still need to be confirmed present on the camera, or bundled the way
    # app/src/CMakeLists.txt's commented-out libssl/libcrypto COPY lines do.
    ssl crypto
    stdlib dl pthread m rt)

install (TARGETS ${TARGET_LIB} DESTINATION ${CMAKE_HOME_DIRECTORY}/libs/${TARGET_LIB})
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/manifests/ DESTINATION ${CMAKE_HOME_DIRECTORY}/libs/${TARGET_LIB} FILES_MATCHING PATTERN "*manifest.json")
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/manifests/ DESTINATION ${CMAKE_HOME_DIRECTORY}/res/models FILES_MATCHING PATTERN "*instance*.json" PATTERN "*attribute*.json")
