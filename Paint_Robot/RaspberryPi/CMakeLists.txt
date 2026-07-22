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


