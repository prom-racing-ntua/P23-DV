cmake_minimum_required(VERSION 3.8)
project(lidar_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)

# executable commands
add_executable(${PROJECT_NAME} src/main.cpp src/lidar.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp sensor_msgs custom_msgs)

# install excutables to lib directory
install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

# install other directories to share directory
install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

# directories to include
include_directories(
  include
  sensor_msgs
  custom_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
