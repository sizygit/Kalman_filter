cmake_minimum_required(VERSION 3.0.0)
project(kalman VERSION 0.1.0)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})

add_definitions(-Wall -g)


set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_FIND_DEBUG_MODE DEBUG)

include_directories("E:/other/eigen-3.4.0/eigen")
# 将源代码添加到此项目的可执行文件。

add_executable(kalman src/main.cpp src/kalman_filter.cpp)


