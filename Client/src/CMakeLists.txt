cmake_minimum_required(VERSION 3.16)

project(RoadPainterQml VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2 Network Gui)

# MinGW용 OpenCV 4.12 (팀 공유 빌드).
#
# ⚠️ 머신마다 다르다. MSYS2 로 대체한 머신에서는 아래 줄로 바꿔 쓴다:
#       set(OpenCV_DIR "C:/msys64/mingw64/lib/cmake/opencv4")   # OpenCV 4.13.0
#    Qt 의 MinGW 13.1.0 과 MSYS2 mingw64 GCC 는 별개 배포판이라 ABI 가 완전히 같다는
#    보장은 없지만, 양쪽 다 빌드+링크까지 통과한 것이 확인돼 있다.
#
# ⚠️ 이 경로를 바꾸면 번들 FFmpeg 버전도 같이 바뀐다. video_worker 의 RTSP 타임아웃
#    옵션 이름이 FFmpeg 5.0 을 기점으로 stimeout → timeout 으로 달라지므로,
#    코드가 실행 시점에 avformat 버전을 보고 고르게 돼 있다 (video_worker.cpp 참고).
set(OpenCV_DIR "C:/opencv/mingw")
find_package(OpenCV REQUIRED)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(RoadPainterQml
    main.cpp
    backend.h backend.cpp
    videoview.h videoview.cpp
    video_worker.h video_worker.cpp
    # 4채널 미리보기 (PNM). 경량 캡처 + 타일 — video_worker/VideoView 와 별개다
    # (미리보기에 ArUco 검출과 3000줄짜리 작도 뷰를 재사용하면 CPU 가 녹는다)
    preview_worker.h preview_worker.cpp
    videofilters.h
    channel_tile.h channel_tile.cpp
    serverclient.h serverclient.cpp
    routeplan.h
    motionprogram.h
    paintgeometry.h
    motionsim.h
    camcalib.h
    strokefont.h
    fonts.qrc
    assets.qrc
)

set_source_files_properties(Theme.qml PROPERTIES QT_QML_SINGLETON_TYPE TRUE)

qt_add_qml_module(RoadPainterQml
    URI RoadPainter
    VERSION 1.0
    QML_FILES
        Main.qml
        Theme.qml
        AppButton.qml
        VideoPane.qml
        LoginPage.qml
        SignupDialog.qml
        MainPage.qml
        ChannelGrid.qml
        PresetTile.qml
        ScaleRuler.qml
        DockPanel.qml
        PathThumb.qml
        JobHistoryDialog.qml
)

# MinGW 전용: COFF 오브젝트는 섹션이 32767개를 넘을 수 없다.
# MainPage.qml 처럼 큰 QML 은 qmlcache 가 만들어내는 .cpp 가 이 한계를 넘겨
#   as.exe: too many sections (33489) / Fatal error: file too big
# 으로 죽는다 (Debug 빌드에서 특히 잘 터진다 — 최적화가 없어 코드가 더 크다).
# big-obj 형식으로 어셈블하면 한계가 사라진다. Ninja/Makefiles 양쪽 다 적용된다.
if (MINGW)
    target_compile_options(RoadPainterQml PRIVATE -Wa,-mbig-obj)
elseif (MSVC)
    target_compile_options(RoadPainterQml PRIVATE /bigobj)   # 같은 문제, MSVC 표기
endif()

# 실행 파일은 처음부터 dist/RoadPainter.exe 로 만든다.
#
# 예전에는 build-smoke/RoadPainterQml.exe 를 만든 뒤 빌드 .bat 가 dist/RoadPainter.exe
# 로 복사했다. 그러면 겉보기에 똑같은 exe 가 두 개 생기는데, 옆에 Qt/OpenCV DLL 이
# 깔려 있는 건 dist 쪽 하나뿐이다. RoadPainterQml.exe 를 직접 실행하면
# "libopencv_core4120.dll이 없어 코드 실행을 진행할 수 없습니다" 로 죽는다.
# 빌드 산출물을 하나로 두면 그 함정 자체가 없어진다.
set_target_properties(RoadPainterQml PROPERTIES
    OUTPUT_NAME              "RoadPainter"
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/dist"
)

# ToolTip is loaded lazily through the generic QtQuick.Controls module even
# though this application imports the Basic style directly. windeployqt 6.11
# does not detect that runtime-only dependency, leaving ToolTip creation to fail
# with "qtquickcontrols2plugin not found". Keep the two module entry files in
# every build output so the packaged executable works on machines without Qt.
get_target_property(_qt_qmake_executable Qt6::qmake IMPORTED_LOCATION)
get_filename_component(_qt_bin_dir "${_qt_qmake_executable}" DIRECTORY)
get_filename_component(_qt_prefix_dir "${_qt_bin_dir}" DIRECTORY)
set(_qt_controls_qml_dir "${_qt_prefix_dir}/qml/QtQuick/Controls")
set(_app_controls_qml_dir "$<TARGET_FILE_DIR:RoadPainterQml>/qml/QtQuick/Controls")
add_custom_command(TARGET RoadPainterQml POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E make_directory "${_app_controls_qml_dir}"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_qt_controls_qml_dir}/qmldir"
            "${_app_controls_qml_dir}/qmldir"
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${_qt_controls_qml_dir}/qtquickcontrols2plugin.dll"
            "${_app_controls_qml_dir}/qtquickcontrols2plugin.dll"
    VERBATIM
)

target_include_directories(RoadPainterQml PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})

target_link_libraries(RoadPainterQml PRIVATE
    Qt6::Quick
    Qt6::QuickControls2
    Qt6::Network
    Qt6::Gui
    ${OpenCV_LIBS}
)

option(ROADPAINTER_BUILD_TESTS "Build deterministic path-generation regression tests" OFF)
if (ROADPAINTER_BUILD_TESTS)
    enable_testing()
    add_executable(motionprogram_tests tests/motionprogram_tests.cpp)
    target_include_directories(motionprogram_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
    target_link_libraries(motionprogram_tests PRIVATE Qt6::Core Qt6::Gui)
    add_test(NAME motionprogram_tests COMMAND motionprogram_tests)
endif()
