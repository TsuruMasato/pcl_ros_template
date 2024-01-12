cmake_minimum_required(VERSION 3.0.0)
project(edge_detector_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/edge_detector.cpp)

add_executable(edge_detector_node src/edge_detector_node.cpp)

target_link_libraries(edge_detector_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS edge_detector_ros
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
