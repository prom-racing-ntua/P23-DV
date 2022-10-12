cmake_minimum_required(VERSION 3.0.2)
project(path_planning_code)

add_compile_options(-std=c++11)

set(PROJECT_DEPS
  roscpp
  std_msgs
  fsd_common_msgs
  )

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  fsd_common_msgs
  geometry_msgs
  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
)

# Each node in the package must be declared like this
add_executable(${PROJECT_NAME}
    src/path_planning_handler.cpp
    src/path_planning.cpp
    src/main.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )