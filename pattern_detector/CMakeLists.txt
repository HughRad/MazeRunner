cmake_minimum_required(VERSION 3.0.2)
project(pattern_detector)

## Find catkin dependencies
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
  roslib
)

## Find OpenCV
find_package(OpenCV REQUIRED)

## Declare catkin package
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs image_transport cv_bridge
  DEPENDS OpenCV
)

## Specify include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

## Declare the main node executable
add_executable(pattern_detector_node
  src/pattern_detector_node.cpp
  src/pattern_detector.cpp
)

## Link the main node executable
target_link_libraries(pattern_detector_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Declare the test node executable
add_executable(pattern_detector_test
  src/pattern_detector_test.cpp
  src/pattern_detector.cpp
)

## Link the test node executable
target_link_libraries(pattern_detector_test
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

## Declare the test image generator executable
add_executable(generate_test_images
  src/generate_test_images.cpp
)

## Link the test image generator executable
target_link_libraries(generate_test_images
  ${OpenCV_LIBRARIES}
)

## Install targets
install(TARGETS pattern_detector_node pattern_detector_test generate_test_images
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## Create test_images directory if it doesn't exist
file(MAKE_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/test_images)