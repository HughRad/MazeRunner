cmake_minimum_required(VERSION 3.1.3)
project(maze_runner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  ur3_moveit_config 
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_commander
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_commander_INCLUDE_DIRS}
)

## Declare a C++ executable for each node
add_executable(maze_runner src/robot_control.cpp)
add_executable(waypoint_test src/waypoint_test.cpp)
add_executable(robot_sim src/robot_control.cpp)

## Add dependencies to the executables
add_dependencies(robot_sim 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_core_EXPORTED_TARGETS}
  ${moveit_ros_planning_interface_EXPORTED_TARGETS}
  ${moveit_commander_EXPORTED_TARGETS}
)

add_dependencies(maze_runner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_core_EXPORTED_TARGETS}
  ${moveit_ros_planning_interface_EXPORTED_TARGETS}
  ${moveit_commander_EXPORTED_TARGETS}
)

add_dependencies(waypoint_test 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_core_EXPORTED_TARGETS}
  ${moveit_ros_planning_interface_EXPORTED_TARGETS}
  ${moveit_commander_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
## maze solver is added as a library rather than an executable
add_library(maze_solver_lib src/maze_solver.cpp)  

target_link_libraries(robot_sim
  maze_solver_lib
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

target_link_libraries(maze_runner
  maze_solver_lib
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

target_link_libraries(waypoint_test
  maze_solver_lib
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS maze_runner waypoint_test robot_sim
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/maze_runner.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
