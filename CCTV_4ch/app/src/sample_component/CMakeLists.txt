set(TARGET_LIB sample_component)
set(THIRD_PARTY ${CMAKE_HOME_DIRECTORY}/third_party)

add_library(${TARGET_LIB} MODULE
    sample_component.cc
    aruco_processor.cc
    pose_sender.cc
    intrinsics_calib.cc
    homography_mapper.cc
    dyn_roi.cc)

target_include_directories(${TARGET_LIB} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/includes
    ${THIRD_PARTY}/opencv-aarch64/include/opencv4
    # p_raw_image.h pulls in raw_image.h, which lives under the NPU API tree.
    ${WISENET_PRODUCT_RELEASE_PATH}/lib/npu_api/includes)

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
    stdlib dl pthread m rt)

install (TARGETS ${TARGET_LIB} DESTINATION ${CMAKE_HOME_DIRECTORY}/libs/${TARGET_LIB})
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/manifests/ DESTINATION ${CMAKE_HOME_DIRECTORY}/libs/${TARGET_LIB} FILES_MATCHING PATTERN "*manifest.json")
install (DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/manifests/ DESTINATION ${CMAKE_HOME_DIRECTORY}/res/models FILES_MATCHING PATTERN "*instance*.json" PATTERN "*attribute*.json")
