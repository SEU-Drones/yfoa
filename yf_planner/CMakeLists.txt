cmake_minimum_required(VERSION 3.0.2)
project(yf_planner)

ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)

# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  gridmap
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yf_planner
  CATKIN_DEPENDS gridmap 
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(NLopt REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

include_directories("/usr/include/eigen3")

add_library(yf_planner
  include/HybirdAstar.h
  include/PathNlopt.h
  include/UniformBspline.h
  src/HybirdAstar.cpp
  src/PathNlopt.cpp
  src/UniformBspline.cpp
)

target_link_libraries(yf_planner
  ${catkin_LIBRARIES}
  ${NLOPT_LIBRARIES}
  ${PCL_LIBRARIES}
)

