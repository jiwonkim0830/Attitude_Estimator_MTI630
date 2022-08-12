cmake_minimum_required(VERSION 3.10)
project(mti630 LANGUAGES CXX)

add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS_RELEASE_INIT "-Wall")

find_package(Eigen3 REQUIRED)

##########  for ROS  ############
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  std_msgs
  sensor_msgs
  visualization_msgs
  tf
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

link_libraries(${catkin_LIBRARIES})

##########  for ROS end  ############

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ../../include
  ../../include/xsensdeviceapi
)

link_libraries(
  xsensdeviceapi
  xstypes
  pthread
)

link_directories(
  ../../lib
)

add_executable(main src/main.cpp)