cmake_minimum_required(VERSION 3.10)
project(SystemMonitor)

set(CMAKE_CXX_STANDARD 11)

# Sadece monitor.cpp dosyasını derle
add_executable(SystemMonitor monitor.cpp)