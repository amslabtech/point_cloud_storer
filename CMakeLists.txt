cmake_minimum_required(VERSION 3.0.2)
project(point_cloud_storer)

SET(CMAKE_CXX_FLAGS "-std=c++14 -O2 -g -Wall ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  nav_msgs
  sensor_msgs
  tf
)

find_package(OpenMP)
if (OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Eigen3 REQUIRED COMPONENTS system)
find_package(PCL 1.8 REQUIRED)
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES point_cloud_storer
  CATKIN_DEPENDS message_filters nav_msgs roscpp sensor_msgs tf
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(point_cloud_storer src/point_cloud_storer_node.cpp src/point_cloud_storer.cpp)
add_executable(transform_point_cloud src/transform_point_cloud_node.cpp src/transform_point_cloud.cpp)
target_link_libraries(point_cloud_storer
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(transform_point_cloud
  ${catkin_LIBRARIES}
)
