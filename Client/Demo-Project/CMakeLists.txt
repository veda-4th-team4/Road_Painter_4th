cmake_minimum_required(VERSION 3.16)

project(DemoProject VERSION 0.1 LANGUAGES CXX)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# MinGW 킷용으로 소스에서 직접 빌드한 OpenCV 4.12 (objdetect 에 ArUco 포함).
# 기존 C:/opencv/build 는 MSVC(vc16) 전용 바이너리라 MinGW 킷과 ABI 불일치 → 사용 불가.
set(OpenCV_DIR "C:/opencv/mingw")
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(PROJECT_SOURCES
        main.cpp
        mainwindow.cpp
        mainwindow.h
        mainwindow.ui
)

if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(DemoProject
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
        resources.qrc
        video_worker.h video_worker.cpp
        videowidget.h videowidget.cpp
        videosettingsdialog.h videosettingsdialog.cpp videosettingsdialog.ui
        PathSaveDialog.h

        channelsettings.h channelsettings.cpp channelsettings.ui

        settingdialog.h settingdialog.cpp settingdialog.ui
        robotsettings.h robotsettings.cpp robotsettings.ui
        labdialog.h labdialog.cpp labdialog.ui


    )
# Define target properties for Android with Qt 6 as:
#    set_property(TARGET DemoProject APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
#                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
else()
    if(ANDROID)
        add_library(DemoProject SHARED
            ${PROJECT_SOURCES}
        )
# Define properties for Android with Qt 5 after find_package() calls as:
#    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
    else()
        add_executable(DemoProject
            ${PROJECT_SOURCES}
        )
    endif()
endif()

target_link_libraries(DemoProject PRIVATE Qt${QT_VERSION_MAJOR}::Widgets
    ${OpenCV_LIBS}
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
if(${QT_VERSION} VERSION_LESS 6.1.0)
  set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.DemoProject)
endif()
set_target_properties(DemoProject PROPERTIES
    ${BUNDLE_ID_OPTION}
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

include(GNUInstallDirs)
install(TARGETS DemoProject
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(DemoProject)
endif()
