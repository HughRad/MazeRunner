cmake_minimum_required(VERSION 3.1.3)
project(maze_runner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  rospy
  roscpp
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  ur3_moveit_config
  cv_bridge
  sensor_msgs
  std_msgs
  std_srvs
  tf2_geometry_msgs
)


find_package(OpenCV REQUIRED)
###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_commander moveit_servo tf2_geometry_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${moveit_core_INCLUDE_DIRS}
  ${moveit_ros_planning_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
  ${moveit_commander_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)


## Declare a C++ executable for each node
add_executable(maze_runner src/robot_control.cpp)
add_executable(robot_sim src/robot_control.cpp)
add_executable(maze_visualizer src/maze_visualizer.cpp)


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

add_dependencies(maze_visualizer 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  ${moveit_core_EXPORTED_TARGETS}
  ${moveit_ros_planning_interface_EXPORTED_TARGETS}
  ${moveit_commander_EXPORTED_TARGETS}
)

## Specify libraries to link a library or executable target against
## maze solver and image processing is added as a library rather than an executable
add_library(maze_solver_lib src/maze_solver.cpp)  
add_library(image_processing src/image_processing.cpp)

target_link_libraries(robot_sim
  maze_solver_lib
  image_processing
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${OpenCV_LIBS}
)


target_link_libraries(maze_runner
  maze_solver_lib
  image_processing
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(maze_visualizer
  maze_solver_lib
  image_processing
  ${catkin_LIBRARIES}
  ${moveit_core_LIBRARIES}
  ${moveit_ros_planning_interface_LIBRARIES}
  ${OpenCV_LIBS}
)
#############
## Install ##
#############

## Mark executables for installation
install(TARGETS maze_runner robot_sim
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
  launch/maze_runner.launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts for gui
catkin_install_python(PROGRAMS
  scripts/gui.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

