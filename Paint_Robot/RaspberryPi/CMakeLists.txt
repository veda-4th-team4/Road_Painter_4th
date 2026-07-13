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

# 2) Robot Simulator Client Target
add_executable(robot_sim src/robot_sim.cpp)
target_link_libraries(robot_sim OpenSSL::SSL OpenSSL::Crypto pthread)
