cmake_minimum_required(VERSION 3.0.0)
project(edge_detector_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
)

# if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
#      set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
# endif()
find_package(PCL 1.8 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

add_library(edge_detector src/edge_detector.cpp)

target_link_libraries(edge_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(edge_detector_node src/edge_detector_node.cpp)

target_link_libraries(edge_detector_node
  edge_detector
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS edge_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
