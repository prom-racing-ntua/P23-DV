cmake_minimum_required(VERSION 3.5)
project(slam)

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
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(TBB REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# GTSAM
find_package(GTSAMCMakeTools REQUIRED)
find_package(GTSAM REQUIRED)

include_directories(
  include
  include/lifecycle
  ${GTSAM_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/slam_handler.cpp src/slam.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp visualization_msgs custom_msgs ament_index_cpp tf2_geometry_msgs)
target_link_libraries(${PROJECT_NAME} gtsam)

add_executable(read_files src/slam_from_file.cpp src/slam.cpp)
ament_target_dependencies(read_files rclcpp visualization_msgs custom_msgs ament_index_cpp tf2_geometry_msgs)
target_link_libraries(read_files gtsam)

add_executable(test_run src/test_visual.cpp)
ament_target_dependencies(test_run rclcpp visualization_msgs ament_index_cpp tf2_geometry_msgs)

# Lifecycle Node
add_executable(lifecycle_${PROJECT_NAME} src/lifecycle/lifecycle_slam_handler.cpp src/lifecycle/lifecycle_slam.cpp src/lifecycle/lifecycle_states.cpp)
ament_target_dependencies(lifecycle_${PROJECT_NAME} rclcpp rclcpp_lifecycle visualization_msgs custom_msgs ament_index_cpp tf2_geometry_msgs)
target_link_libraries(lifecycle_${PROJECT_NAME} gtsam ${catkin_LIBRARIES})

install(TARGETS
  ${PROJECT_NAME}
  read_files
  test_run
  lifecycle_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  test_tracks DataFiles launch config
  DESTINATION share/${PROJECT_NAME}/
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