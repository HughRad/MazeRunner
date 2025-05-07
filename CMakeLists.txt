cmake_minimum_required(VERSION 3.0.2)
project(aruco_tracker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
  aruco_ros
  message_generation
)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge image_transport tf2 tf2_ros tf2_geometry_msgs aruco_ros message_runtime
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_tracker_node
  src/aruco_tracker.cpp
)

target_link_libraries(aruco_tracker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Create snapshot directory in build space
add_custom_command(
  TARGET aruco_tracker_node POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_CURRENT_BINARY_DIR}/snapshot
)

install(TARGETS aruco_tracker_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install the snapshot directory
install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/snapshot/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/snapshot
)

# Install any existing snapshot directory from source
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/snapshot/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/snapshot
  OPTIONAL
)