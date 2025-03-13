cmake_minimum_required(VERSION 3.0.2)
project(maze_runner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

# Explicitly find MoveIt components
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES maze_runner
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS moveit_core moveit_ros_planning_interface
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(robot_control src/robot_control.cpp)

## Add dependencies to the executable
add_dependencies(robot_control 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_core_EXPORTED_TARGETS}
  ${moveit_ros_planning_interface_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(robot_control
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

add_executable(waypoint_test src/waypoint_test.cpp)
target_link_libraries(waypoint_test
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
)

#############
## Install ##
#############

## Mark executables for installation
install(TARGETS robot_control
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/maze_runner.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)