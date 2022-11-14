cmake_minimum_required(VERSION 3.5)
project(graph_slam)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)

add_executable(${PROJECT_NAME} src/graph_slam_handler.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp custom_msgs)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  custom_msgs
)

ament_package()
