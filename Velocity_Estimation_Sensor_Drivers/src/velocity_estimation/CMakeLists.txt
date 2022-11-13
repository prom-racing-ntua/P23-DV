cmake_minimum_required(VERSION 3.8)
project(velocity_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# executable commands
add_executable(${PROJECT_NAME} src/main.cpp src/velocity_estimation.cpp src/velocity_estimation_handler.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp vectornav_msgs custom_msgs)

# install
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

# directories to include
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  vectornav_msgs
  custom_msgs
)

ament_package()
