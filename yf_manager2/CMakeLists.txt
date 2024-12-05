cmake_minimum_required(VERSION 3.0.2)
project(yf_manager2)

ADD_COMPILE_OPTIONS(-std=c++11)
ADD_COMPILE_OPTIONS(-std=c++14)

# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3 -Wall -pthread") # -Wextra -Werror
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  message_filters
  tf
  message_generation
  mavros_msgs
  cv_bridge
  gridmap
  yf_planner
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

include_directories("/usr/include/eigen3")

include_directories(
  SYSTEM
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_message_files(
  FILES

  WayPoints.msg
  Bspline.msg
  Spline.msg
  Splines.msg

)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yf_manager2
  CATKIN_DEPENDS message_runtime gridmap yf_planner
)

add_executable(PlanFSM_node2
  include/PlanFSM.h
  src/PlanFSM.cpp 
  src/PlanFSM_node.cpp
  )
target_link_libraries(PlanFSM_node2
  ${catkin_LIBRARIES}
  )

add_executable(MissionXYZ_node2
  include/MissionXYZ.h
  src/MissionXYZ.cpp 
  src/MissionXYZ_node.cpp
  )
target_link_libraries(MissionXYZ_node2
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
