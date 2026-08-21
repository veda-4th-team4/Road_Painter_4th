cmake_minimum_required(VERSION 3.10)
project(RobotPainterController CXX)

set(CMAKE_CXX_STANDARD 17)

# Locate OpenSSL library on target system
find_package(OpenSSL REQUIRED)

# Include directories
include_directories(include ${OPENSSL_INCLUDE_DIR})

# 1) Robot Main Executable Target
set(SOURCES
    src/main.cpp
    src/SerialManager.cpp
    src/NetworkManager.cpp
    src/PathFollower.cpp
    src/ImuManager.cpp
    src/LedStripManager.cpp
)
add_executable(robot_exec ${SOURCES})
target_link_libraries(robot_exec wiringPi OpenSSL::SSL OpenSSL::Crypto pthread)

# 2) Stepper Motor Drive Test Tool
set(MOTOR_TEST_SOURCES
    src/motor_test.cpp
    src/SerialManager.cpp
)
add_executable(motor_test ${MOTOR_TEST_SOURCES})
target_link_libraries(motor_test wiringPi pthread)

# 3) Rear Nozzle Offset Arc Motion Test Tool
set(ARC_TEST_SOURCES
    src/arc_test.cpp
    src/SerialManager.cpp
)
add_executable(arc_test ${ARC_TEST_SOURCES})
target_link_libraries(arc_test wiringPi pthread)

# 4) Hardware Distance & Angle Calibration Test Tool
set(CALIB_TEST_SOURCES
    src/calibration_test.cpp
    src/SerialManager.cpp
    src/PathFollower.cpp
    src/ImuManager.cpp
)
add_executable(calibration_test ${CALIB_TEST_SOURCES})
target_link_libraries(calibration_test wiringPi pthread)

# 5) Tower Lamp (WS2812B) Status LED Test Tool
set(LED_TEST_SOURCES
    src/led_test.cpp
    src/LedStripManager.cpp
)
add_executable(led_test ${LED_TEST_SOURCES})
target_link_libraries(led_test pthread)

# 6) Odometry Driving Homography Calibration Path Test Tool
set(CALIB_PATH_TEST_SOURCES
    src/calib_path_test.cpp
    src/SerialManager.cpp
    src/PathFollower.cpp
    src/ImuManager.cpp
)
add_executable(calib_path_test ${CALIB_PATH_TEST_SOURCES})
target_link_libraries(calib_path_test wiringPi pthread)


