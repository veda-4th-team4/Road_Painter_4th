cmake_minimum_required(VERSION 3.10)
project(RobotPainterController CXX)

set(CMAKE_CXX_STANDARD 17)

# Include directories
include_directories(include)

# Source files
set(SOURCES
    src/main.cpp
    src/SerialManager.cpp
    src/NetworkManager.cpp
    src/PathFollower.cpp
)

# Executable target
add_executable(robot_exec ${SOURCES})

# Link wiringPi and thread libraries
target_link_libraries(robot_exec wiringPi pthread)
