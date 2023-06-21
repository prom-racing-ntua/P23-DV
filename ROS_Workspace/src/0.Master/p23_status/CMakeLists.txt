cmake_minimum_required(VERSION 3.8)
project(p23_status)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)

include_directories(
  include
  include/common
  ${rclcpp_action_INCLUDE_DIRS}
)


# --- Common Lib
add_library(p23_common INTERFACE)

install(
  TARGETS p23_common
  EXPORT p23_common
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(p23_common HAS_LIBRARY_TARGET)
# ament_export_include_directories(include)

install(DIRECTORY include/common/ DESTINATION include/)


# --- P23 Node
add_executable(${PROJECT_NAME} src/main.cpp src/p23_status_node.cpp src/p23_status_callbacks.cpp src/p23_status_action.cpp)
target_link_libraries(${PROJECT_NAME} p23_common)
ament_target_dependencies(${PROJECT_NAME} rclcpp custom_msgs rclcpp_action std_srvs)

install(DIRECTORY
  config launch
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
