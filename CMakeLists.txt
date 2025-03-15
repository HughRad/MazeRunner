cmake_minimum_required (VERSION 3.0.0)

project(maze_runner)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(image_processing src/main.cpp src/image_processing.cpp)