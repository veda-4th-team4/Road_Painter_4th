cmake_minimum_required(VERSION 3.21)

project(RoadPainter VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 6.5 REQUIRED COMPONENTS Quick QuickControls2 ShaderTools)

qt_standard_project_setup(REQUIRES 6.5)

qt_add_executable(appRoadPainter
    src/main.cpp
)

set_source_files_properties(qml/Theme.qml PROPERTIES
    QT_QML_SINGLETON_TYPE TRUE
)

qt_add_qml_module(appRoadPainter
    URI RoadPainter
    VERSION 1.0
    QML_FILES
        qml/Main.qml
        qml/Theme.qml
        qml/VideoViewport.qml
        qml/CctvScene.qml
        qml/PathOverlay.qml
        qml/RobotMarker.qml
        qml/ChannelStrip.qml
        qml/PresetDialog.qml
        qml/PathSaveDialog.qml
        qml/PanelSection.qml
        qml/LabeledSlider.qml
        qml/SideToolButton.qml
        qml/AppButton.qml
        qml/AppSwitch.qml
        qml/Toast.qml
    SOURCES
        src/pathmodel.h src/pathmodel.cpp
        src/presetlibrary.h src/presetlibrary.cpp
        src/robotsimulator.h src/robotsimulator.cpp
        src/filtersettings.h src/filtersettings.cpp
)

qt_add_shaders(appRoadPainter "roadpainter_shaders"
    PREFIX "/shaders"
    BASE "shaders"
    FILES
        shaders/videofilter.frag
)

set_target_properties(appRoadPainter PROPERTIES
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_include_directories(appRoadPainter PRIVATE src)

target_link_libraries(appRoadPainter
    PRIVATE Qt6::Quick Qt6::QuickControls2
)

install(TARGETS appRoadPainter
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
