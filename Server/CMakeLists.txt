# 서버 RPi 빌드.
#   sudo apt install libssl-dev cmake g++
#   cmake -B build && cmake --build build
#   ./build/server_rpi        (certs/ 에 server.crt, server.key 필요)
cmake_minimum_required(VERSION 3.10)
project(server_rpi CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenSSL REQUIRED)

add_executable(server_rpi
    src/main.cpp
    src/net/tls_server.cpp
)
target_include_directories(server_rpi PRIVATE include src)
target_link_libraries(server_rpi PRIVATE OpenSSL::SSL OpenSSL::Crypto)
