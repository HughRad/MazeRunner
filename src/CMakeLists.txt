cmake_minimum_required(VERSION 3.10)
project(mazesolver_test)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS geometry_msgs roscpp)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(maze_solver main.cpp maze_solver.cpp)

target_link_libraries(maze_solver ${catkin_LIBRARIES})